use crate::control::{quick_turn_time_with_target_omega, quick_turn_with_target_omega};
use crate::missile::{MissileGuidance, TargetTelemetry};
use oort_api::prelude::*;
use std::cell::RefCell;
use std::rc::Rc;

const SALVO_SIZE: usize = 2;
use crate::aim::FireControl;
use crate::physics::KinematicState;
use crate::radar::{Contact, DefaultScanSliceGenerator, RadarController};
use crate::radio::{RadioManager, SecureRadio};

fn can_missile_intercept(missile: &KinematicState, target: &KinematicState) -> bool {
    let t_now = current_tick();

    // Extrapolate the missile's position and velocity to the current tick,
    // using its scanned acceleration to account for movement and acceleration between scans.
    let missile_pos = missile.position_at(t_now);
    let missile_vel = missile.velocity_at(t_now);

    let r0_vec = target.position - missile_pos;
    let r0 = r0_vec.length();
    if r0 < 1e-6 {
        return true;
    }

    let u0 = r0_vec / r0;
    let w0 = Vec2::new(-u0.y, u0.x);
    let v_rel = target.velocity - missile_vel;
    let v_c = -v_rel.dot(u0);

    if v_c <= 0.0 {
        return false;
    }

    let t_go = r0 / v_c;

    let missile_stats = Class::Missile.default_stats();
    let max_lat_acc = missile_stats.max_lateral_acceleration;

    let v_rel_perp = v_rel.dot(w0);
    let d_steer = v_rel_perp.abs() * t_go;

    // Calculate max bullet displacement at t_go factoring in detonation 0.2s before impact
    let explode_time = 0.2;
    let explode_vel = 750.0;
    let d_max = if t_go > explode_time {
        0.5 * max_lat_acc * (t_go - explode_time).powi(2) + explode_vel * explode_time
    } else {
        explode_vel * t_go
    };

    let is_threat = d_max >= d_steer;

    // --- Diagnostic Debug Drawings ---
    // The center of the missile's steerable zone is its ballistic trajectory (no lateral thrust)
    let p_missile_center = missile_pos + missile_vel * t_go;
    let p_target = target.position + target.velocity * t_go;

    // Draw ballistic path in orange
    draw_line(missile_pos, p_missile_center, rgb(255, 128, 0));
    // Draw target path in yellow
    draw_line(target.position, p_target, rgb(255, 255, 0));

    // Draw steerable zone circle (including bullet spread) centered at p_missile_center in green
    draw_polygon(p_missile_center, d_max, 32, 0.0, rgb(0, 255, 0));

    {
        let color = if is_threat {
            rgb(255, 0, 0)
        } else {
            rgb(0, 255, 0)
        };
        draw_line(missile_pos, p_target, rgb(255, 0, 0));
        draw_diamond(p_target, 20.0, rgb(255, 0, 0));
        draw_text!(missile_pos + vec2(0.0, 30.0), color, "t_go:{:.1}s", t_go);
    }

    is_threat
}

pub fn filter_missile_threats(contacts: &[Contact], our_pos: Vec2, our_vel: Vec2) -> Vec<u32> {
    let mut missile_threats = Vec::new();
    let our_kin = KinematicState::new(
        Class::Fighter,
        our_pos,
        our_vel,
        Vec2::new(0.0, 0.0),
        current_tick(),
    );
    for c in contacts.iter().filter(|c| c.class == Class::Missile) {
        let r = c.current_position() - our_pos;
        let dist = r.length();
        if dist > 0.0 {
            let v_rel = c.current_velocity() - our_vel;
            let closing = -r.dot(v_rel) / dist;
            if closing > 0.0 {
                if can_missile_intercept(&c.kinematic, &our_kin) {
                    let v_rel_len_sq = v_rel.dot(v_rel);
                    if v_rel_len_sq > 0.0 {
                        let t_closest = -r.dot(v_rel) / v_rel_len_sq;
                        if t_closest > 0.0 && t_closest <= 5.0 {
                            let closest_pos = r + t_closest * v_rel;
                            let closest_approach_dist = closest_pos.length();
                            if closest_approach_dist <= 150.0 {
                                missile_threats.push((c.id, dist));
                            }
                        }
                    }
                }
            }
        }
    }
    missile_threats.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
    missile_threats.iter().take(3).map(|&(id, _)| id).collect()
}

pub struct Ship {
    radar_controller: RadarController,
    missile_guidance: MissileGuidance,

    missile_radio: SecureRadio,
    fighter_radio: SecureRadio,
    fighter_target_id: Option<u32>,
    fighter_msgs_received: u32,
    fighter_last_known_target_pos: Option<Vec2>,
    fighter_last_known_target_vel: Option<Vec2>,

    // Orbit and movement fields
    orbit_direction: f64,
    target_orbit_speed_fraction: f64,
    num_direction_changes: u32,

    // Point Defense tracking state
    pd_target_id: Option<u32>,
    pd_bullets_fired: u32,

    // Magazine reload tracking state
    gun0: FireControl,
    gun1: FireControl,
    force_reload: bool,

    target_pos: Rc<RefCell<Option<Vec2>>>,
    ticks_since_reversal: u32,
    salvo_missiles_fired: usize,
    salvo_aim_point: Option<Vec2>,
    salvo_arrival_tick: u32,
    ticks_since_missile_fired: u32,
}

impl Ship {
    pub fn new() -> Ship {
        let mut rc = RadarController::new();
        let target_pos = Rc::new(RefCell::new(None));
        let mut generator = DefaultScanSliceGenerator::new(0.6, 20000.0);
        generator.target_pos = Some(target_pos.clone());
        generator.biased_scan_width = 60.0f64.to_radians();
        rc.slice_generator = Box::new(generator);

        let radio_manager = Rc::new(RefCell::new(RadioManager::new()));
        let missile_radio = SecureRadio::new(1337, 0, radio_manager.clone());
        let fighter_radio = SecureRadio::new(1337, 4, radio_manager);

        let mut mg = MissileGuidance::new();
        mg.target_channel = 3;
        mg.secure_radio = Some(missile_radio.clone());

        Ship {
            radar_controller: rc,
            missile_guidance: mg,
            missile_radio,
            fighter_radio,
            fighter_target_id: None,
            fighter_msgs_received: 0,
            fighter_last_known_target_pos: None,
            fighter_last_known_target_vel: None,
            orbit_direction: if rand(0.0, 1.0) < 0.5 { 1.0 } else { -1.0 },
            target_orbit_speed_fraction: rand(0.5, 0.95),
            num_direction_changes: 0,
            pd_target_id: None,
            pd_bullets_fired: 0,
            gun0: FireControl::from_gun(0),
            gun1: FireControl::from_gun(1),
            force_reload: false,
            target_pos,
            ticks_since_reversal: 9999,
            salvo_missiles_fired: 0,
            salvo_aim_point: None,
            salvo_arrival_tick: 0,
            ticks_since_missile_fired: 9999,
        }
    }

    pub fn tick(&mut self) {
        if class() == Class::Missile {
            self.missile_guidance.tick();
            return;
        }

        self.ticks_since_missile_fired = self.ticks_since_missile_fired.saturating_add(1);

        if class() == Class::Fighter {
            self.ticks_since_reversal = self.ticks_since_reversal.saturating_add(1);
            debug!("Fighter ID: {}", id());
            debug!("Position: {:?}", position());

            // --- Magazine Tracking ---
            debug!("Magazine bullets: {}", self.gun0.bullets_in_magazine());

            // 0. Update priority targets based on point defense, threats, and ships
            let mut priority_freqs = Vec::new();

            // Point defense target (scanned every 4/60 seconds)
            if let Some(pd_id) = self.pd_target_id {
                priority_freqs.push((pd_id, 4.0 / 60.0));
            }

            // Other threat missiles incoming in the next 4 seconds (scanned every 1/6th of a second)
            let our_kin = KinematicState::new(
                Class::Fighter,
                position(),
                velocity(),
                Vec2::new(0.0, 0.0),
                current_tick(),
            );
            for c in self
                .radar_controller
                .contacts()
                .iter()
                .filter(|c| c.class == Class::Missile)
            {
                if Some(c.id) == self.pd_target_id {
                    continue;
                }
                let r = c.current_position() - position();
                let dist = r.length();
                if dist > 0.0 {
                    let v_rel = c.current_velocity() - velocity();
                    let closing = -r.dot(v_rel) / dist;
                    if closing > 0.0 {
                        let t_intercept = dist / closing;
                        if t_intercept <= 4.0 {
                            if can_missile_intercept(&c.kinematic, &our_kin) {
                                priority_freqs.push((c.id, 1.0 / 6.0));
                            }
                        }
                    }
                }
            }

            // Non-missile enemy ships (scanned every 1/6th of a second)
            for c in self.radar_controller.contacts().iter() {
                if c.class != Class::Missile && c.class != Class::Torpedo {
                    if Some(c.id) != self.pd_target_id {
                        priority_freqs.push((c.id, 1.0 / 6.0));
                    }
                }
            }
            if let Some(fid) = self.fighter_target_id {
                if !priority_freqs.iter().any(|&(id, _)| id == fid) {
                    priority_freqs.push((fid, 1.0 / 6.0));
                }
            }

            self.radar_controller.priority_target_frequencies = priority_freqs;

            // 1. Update radar scheduler and contact database
            self.radar_controller.update();

            let contacts = self.radar_controller.contacts().to_vec();
            for c in contacts.iter().filter(|c| c.class == Class::Fighter) {
                self.fighter_last_known_target_pos = Some(c.current_position());
                self.fighter_last_known_target_vel = Some(c.current_velocity());
            }

            // --- Incoming Missile Check and Force-Reload Status ---
            let mut incoming_missile_next_5s = false;
            for c in contacts.iter().filter(|c| c.class == Class::Missile) {
                let r = c.current_position() - position();
                let dist = r.length();
                if dist > 0.0 {
                    let v_rel = c.current_velocity() - velocity();
                    let closing = -r.dot(v_rel) / dist;
                    if closing > 0.0 {
                        let t_intercept = dist / closing;
                        if t_intercept <= 5.0 {
                            if can_missile_intercept(&c.kinematic, &KinematicState::self_state()) {
                                incoming_missile_next_5s = true;
                                break;
                            }
                        }
                    }
                }
            }

            let bullets_left = self.gun0.bullets_in_magazine();
            self.force_reload = bullets_left < 15 && bullets_left > 0 && !incoming_missile_next_5s;
            if self.force_reload {
                debug!(
                    "FORCE RELOAD ACTIVE: Firing remaining {} bullets to reload",
                    bullets_left
                );
            }

            if self.fighter_target_id.is_none() {
                let received = self.fighter_radio.receive();
                self.fighter_radio.prepare_receive();

                if let Some(payload) = received {
                    self.fighter_msgs_received += 1;
                    let target_idx = (id() as i32) - 2; // Fighter 2 -> 0, Fighter 3 -> 1, Fighter 4 -> 2
                    if target_idx >= 0
                        && target_idx < 3
                        && self.fighter_msgs_received == (target_idx + 1) as u32
                    {
                        let telemetry = TargetTelemetry::deserialize(&payload);
                        let contact_id = self.radar_controller.add_radio_ping(telemetry);
                        self.fighter_target_id = Some(contact_id);
                        self.fighter_last_known_target_pos = Some(telemetry.position);
                        self.fighter_last_known_target_vel = Some(telemetry.velocity);
                        debug!(
                            "Fighter {} acquired initial target via radio at {:?}",
                            id(),
                            telemetry.position
                        );
                    }
                }
            }

            // 3. Update target tracking
            if let Some(tid) = self.fighter_target_id {
                if let Some(c) = contacts
                    .iter()
                    .find(|c| c.id == tid && c.class == Class::Fighter)
                {
                    self.fighter_last_known_target_pos = Some(c.current_position());
                    self.fighter_last_known_target_vel = Some(c.current_velocity());
                } else {
                    // Target has disappeared. If we previously had it matched, it means it is killed!
                    self.fighter_target_id = None;
                    debug!("Fighter {} target was killed or lost", id());
                }
            }

            // If we don't have a target, try to find one from the radar contacts!
            if self.fighter_target_id.is_none() {
                let mut best_c = None;
                let mut min_d = f64::MAX;
                for c in contacts.iter().filter(|c| c.class == Class::Fighter) {
                    let d = c.current_position().distance(position());
                    if d < min_d {
                        min_d = d;
                        best_c = Some(c.clone());
                    }
                }
                if let Some(c) = best_c {
                    self.fighter_target_id = Some(c.id);
                    self.fighter_last_known_target_pos = Some(c.current_position());
                    self.fighter_last_known_target_vel = Some(c.current_velocity());
                    debug!("Fighter {} acquired target ID {} via radar", id(), c.id);
                }
            }

            // If target is killed/lost, prioritize closest enemy fighter to the target's last known position
            if self.fighter_target_id.is_none() {
                if let Some(last_pos) = self.fighter_last_known_target_pos {
                    let mut best_c = None;
                    let mut min_d = f64::MAX;
                    for c in contacts.iter().filter(|c| c.class == Class::Fighter) {
                        let d = c.current_position().distance(last_pos);
                        if d < min_d {
                            min_d = d;
                            best_c = Some(c.clone());
                        }
                    }
                    if let Some(c) = best_c {
                        self.fighter_target_id = Some(c.id);
                        self.fighter_last_known_target_pos = Some(c.current_position());
                        self.fighter_last_known_target_vel = Some(c.current_velocity());
                        debug!("Fighter {} selected next closest target: ID {}", id(), c.id);
                    }
                }
            }

            // Update the shared target position for the slice generator
            if let Some(tid) = self.fighter_target_id {
                if let Some(c) = contacts
                    .iter()
                    .find(|c| c.id == tid && c.class == Class::Fighter)
                {
                    *self.target_pos.borrow_mut() = Some(c.current_position());
                } else {
                    *self.target_pos.borrow_mut() = None;
                }
            } else {
                *self.target_pos.borrow_mut() = None;
            }

            // Draw a line from our ship through the origin
            draw_line(position(), -position(), rgb(0, 255, 255));

            // Draw a green diamond where the current aim point is for the fighter
            if let Some(aim_point) = self.salvo_aim_point {
                debug!("Aim point {} {}", aim_point.x as i32, aim_point.y as i32);
                draw_diamond(aim_point, 300.0, rgb(0, 255, 0));
            }

            let r_orbit = 15000.0;
            let pos = position();
            let vel = velocity();
            let d = pos.length();
            let u = if d > 1.0 { pos / d } else { vec2(1.0, 0.0) };

            let max_acc = max_forward_acceleration();
            let margin = 0.90; // Reserve 10% of acceleration for radial/tangential control
            let target_speed = (r_orbit * max_acc * margin).sqrt() * 0.75;

            let near_circle = (d - r_orbit).abs() <= 500.0;

            // Check if we just got a solid lock on the enemy fighter for the first time
            let has_solid_lock = self.fighter_target_id.is_some()
                && contacts
                    .iter()
                    .any(|c| Some(c.id) == self.fighter_target_id && c.class == Class::Fighter);
            if has_solid_lock {
                if let Some(c) = contacts
                    .iter()
                    .find(|c| Some(c.id) == self.fighter_target_id && c.class == Class::Fighter)
                {
                    let enemy_pos = c.current_position();

                    // Project our current position onto our orbital circle center (0, 0)
                    let pos_proj = u * r_orbit;

                    // Compute the velocity vector tangent to the circle in our current orbital direction
                    let t = vec2(-u.y, u.x) * self.orbit_direction;
                    let v_tangent = t * target_speed;

                    let to_enemy = enemy_pos - pos_proj;

                    if !near_circle {
                        // While we are not actually near the orbital circle, we should always be running in the direction that takes us further away
                        if to_enemy.dot(v_tangent) > 0.0 {
                            self.orbit_direction = -self.orbit_direction;
                            self.ticks_since_reversal = 0;
                            debug!(
                                "Not near circle: Orbiting toward enemy. Reversing direction to {} to run away.",
                                self.orbit_direction
                            );
                        }
                    }
                }
            }

            let speed = vel.length();

            // Velocity-based orbit direction change: reverse direction when we reach
            // a random fraction of our max orbital velocity (between 0.5 and 0.95).
            // Don't start reversing our direction until we're near the circle.
            let t = vec2(-u.y, u.x) * self.orbit_direction;
            let v_t = vel.dot(t);
            if v_t >= self.target_orbit_speed_fraction * target_speed
                || (current_tick() == 400 && self.ticks_since_reversal >= 150)
            {
                self.orbit_direction = -self.orbit_direction;
                self.target_orbit_speed_fraction = rand(0.5, 0.95);
                self.num_direction_changes += 1;
                self.ticks_since_reversal = 0;
                debug!(
                    "Velocity-based change (near circle): Orbit direction reversed to {}. New fraction threshold: {:.2}",
                    self.orbit_direction, self.target_orbit_speed_fraction
                );
            }

            let t = vec2(-u.y, u.x) * self.orbit_direction;
            let is_reversing_orbit = self.ticks_since_reversal < 300 && vel.dot(t) < target_speed;

            // Calculate desired acceleration for movement based on zone
            let mut acc_cmd;
            if d < r_orbit - 200.0 {
                // Inside: burn at 45 degrees outward relative to the radial direction
                let burn_dir = u.rotate(self.orbit_direction * (TAU / 8.0));
                let desired_vel = burn_dir * target_speed;
                acc_cmd = 1.0 * (desired_vel - vel);
            } else if d > r_orbit + 200.0 {
                // Outside: drive towards the circle on a tangent line
                let phi = (r_orbit / d).asin();
                let tangent_dir = (-u).rotate(-self.orbit_direction * phi);
                let desired_vel = tangent_dir * target_speed;
                acc_cmd = 1.0 * (desired_vel - vel);
            } else {
                // On the circle: standard orbit control
                let centripetal_needed = speed.powi(2) / r_orbit;

                // Radial control: regulate distance to r_orbit
                let e_r = d - r_orbit;
                let v_r = vel.dot(u);
                let kp_r = 0.5;
                let kd_r = 1.0;
                let radial_accel_mag = centripetal_needed + kp_r * e_r + kd_r * v_r;

                // Tangential control: regulate speed to target_speed
                let v_t = vel.dot(t);
                let kp_t = 0.5;
                let tangential_accel_mag = kp_t * (target_speed - v_t);

                acc_cmd = -radial_accel_mag * u + tangential_accel_mag * t;
            }

            if acc_cmd.length() > max_acc {
                acc_cmd = acc_cmd.normalize() * max_acc;
            }

            // Apply acceleration command for movement
            accelerate(acc_cmd);

            // Determine if we want to boost
            let mut want_to_boost = is_reversing_orbit;

            // 5. Point Defense and Gunnery/Missile Aiming
            const BULLET_SPEED: f64 = 1000.0;
            let us_kinematic = KinematicState::self_state();

            // Get all point defense candidates
            let mut pd_candidates = Vec::new();
            for c in contacts.iter().filter(|c| c.class == Class::Missile) {
                let r_m = c.current_position() - position();
                let dist_m = r_m.length();
                if dist_m > 0.0 {
                    let v_rel_m = c.current_velocity() - velocity();
                    let closing_m = -r_m.dot(v_rel_m) / dist_m;
                    if closing_m > 0.0 {
                        let t_intercept = dist_m / closing_m;
                        if t_intercept <= 3.0 && t_intercept >= 0.25 {
                            if can_missile_intercept(&c.kinematic, &us_kinematic) {
                                if let Some(sol) = self.gun0.solve_aim(&c.kinematic, &us_kinematic)
                                {
                                    pd_candidates.push((
                                        c.clone(),
                                        t_intercept,
                                        sol.aim_dir,
                                        sol.omega,
                                    ));
                                }
                            }
                        }
                    }
                }
            }

            // Target selection / switching logic: switch targets after 8 bullets fired
            let mut best_pd_missile: Option<(Contact, f64, Vec2, f64)> = None;
            if !pd_candidates.is_empty() {
                let current_idx = self
                    .pd_target_id
                    .and_then(|id| pd_candidates.iter().position(|(c, _, _, _)| c.id == id));

                if let Some(idx) = current_idx {
                    if self.pd_bullets_fired >= 8 {
                        let other_candidates: Vec<&(Contact, f64, Vec2, f64)> = pd_candidates
                            .iter()
                            .enumerate()
                            .filter(|&(i, _)| i != idx)
                            .map(|(_, item)| item)
                            .collect();

                        if !other_candidates.is_empty() {
                            let best_other = other_candidates
                                .iter()
                                .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
                                .unwrap();
                            best_pd_missile = Some((*best_other).clone());
                            self.pd_target_id = Some(best_other.0.id);
                            self.pd_bullets_fired = 0;
                            debug!(
                                "PD: Switched to next missile ID {} after 8 shots",
                                best_other.0.id
                            );
                        } else {
                            let current = &pd_candidates[idx];
                            best_pd_missile = Some(current.clone());
                        }
                    } else {
                        let current = &pd_candidates[idx];
                        best_pd_missile = Some(current.clone());
                    }
                } else {
                    let best = pd_candidates
                        .iter()
                        .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
                        .unwrap();
                    best_pd_missile = Some(best.clone());
                    self.pd_target_id = Some(best.0.id);
                    self.pd_bullets_fired = 0;
                    debug!("PD: Selected new best missile ID {}", best.0.id);
                }
            } else {
                self.pd_target_id = None;
                self.pd_bullets_fired = 0;
            }

            let mut pd_solution = None;
            if let Some((ref c, _, _, _)) = best_pd_missile {
                if let Some(sol) = self.gun0.solve_aim(&c.kinematic, &us_kinematic) {
                    pd_solution = Some(sol);
                }
            }
            let pd_aim_info = pd_solution
                .as_ref()
                .map(|sol| (sol.aim_dir.angle(), sol.omega));
            if let Some(ref c) = best_pd_missile {
                debug!(
                    "POINT DEFENSE: intercepting incoming missile ID {} in {:.2}s (bullets fired: {})",
                    c.0.id, c.1, self.pd_bullets_fired
                );
                if let Some(ref sol) = pd_solution {
                    draw_line(position(), sol.intercept_pos, rgb(255, 0, 0));
                    draw_diamond(sol.intercept_pos, 20.0, rgb(255, 0, 0));
                }
            }

            let mut gun_solution = None;
            if let Some(contact) = self.fighter_target_id.and_then(|tid| {
                contacts
                    .iter()
                    .find(|c| c.id == tid && c.class == Class::Fighter)
            }) {
                let target_pos = contact.current_position();
                let dp0 = target_pos - position();
                let r_len = dp0.length();
                if r_len > 0.0 {
                    let v_rel = contact.current_velocity() - velocity();
                    let v_c = -v_rel.dot(dp0) / r_len;
                    let t_intercept = r_len / (BULLET_SPEED + v_c.max(0.0));
                    if t_intercept < 3.0 {
                        if let Some(sol) = self.gun0.solve_aim(&contact.kinematic, &us_kinematic) {
                            // Draw line to predicted target position (only if we're not aiming at a point-defense threat)
                            if pd_solution.is_none() {
                                let p_e = sol.intercept_pos;
                                draw_line(position(), p_e, rgb(255, 255, 0));
                            }
                            gun_solution = Some(sol);
                        }
                    }
                }
            }
            let gun_aim_info = gun_solution
                .as_ref()
                .map(|sol| (sol.aim_dir.angle(), sol.omega));

            let mut missile_solution = None;
            if self.gun1.reload_time() <= 10.0 * TICK_LENGTH {
                let target_kinematic = if let Some(contact) =
                    self.fighter_target_id.and_then(|tid| {
                        contacts
                            .iter()
                            .find(|c| c.id == tid && c.class == Class::Fighter)
                    }) {
                    Some(contact.kinematic.clone())
                } else if let Some(last_pos) = self.fighter_last_known_target_pos {
                    let last_vel = self
                        .fighter_last_known_target_vel
                        .unwrap_or(Vec2::new(0.0, 0.0));
                    Some(KinematicState::new(
                        Class::Fighter,
                        last_pos,
                        last_vel,
                        Vec2::new(0.0, 0.0),
                        current_tick(),
                    ))
                } else {
                    None
                };

                if let Some(tk) = target_kinematic {
                    if let Some(sol) = self.gun1.solve_aim(&tk, &us_kinematic) {
                        missile_solution = Some(sol);
                    }
                }
            }
            let missile_aim_info = missile_solution
                .as_ref()
                .map(|sol| (sol.aim_dir.angle(), sol.omega));

            // Calculate desired heading and target omega using the priority logic
            let (desired_heading, target_omega_opt) = self.compute_desired_heading(
                pd_aim_info,
                gun_aim_info,
                missile_aim_info,
                acc_cmd,
                is_reversing_orbit,
            );

            self.log_pd_aim_inputs(
                pd_aim_info,
                desired_heading,
                target_omega_opt,
                &best_pd_missile,
                &us_kinematic,
            );

            // Fire main gun (slot 0)
            if self.force_reload {
                self.gun0.fire();
            } else if let Some(sol) = &pd_solution {
                let diff = angle_diff(heading(), sol.aim_dir.angle());
                if self.gun0.reload_time() == 0.0 && diff.abs() < 0.5f64.to_radians() {
                    self.gun0.fire_at(sol);
                    self.pd_bullets_fired += 1;
                }
            } else if self.fighter_target_id.is_some() {
                // Normal gun firing logic against enemy ship
                if let Some(sol) = &gun_solution {
                    let diff = angle_diff(heading(), sol.aim_dir.angle());
                    if self.gun0.reload_time() <= 5.0 * TICK_LENGTH
                        && diff.abs() < 0.15f64.to_radians()
                    {
                        self.gun0.fire_at(sol);
                    }
                }
            }

            // Determine if we should disable boost while aligning for weapons.
            // When reversing orbit, we only prioritize point defense; we ignore normal gun and missile aiming.
            let gun0_solution = if is_reversing_orbit {
                pd_aim_info
            } else {
                pd_aim_info.or(gun_aim_info)
            };
            let aiming_gun0 = if let Some((angle, omega)) = gun0_solution {
                let turn_time = if max_angular_acceleration() < 1e-6 {
                    0.0
                } else {
                    quick_turn_time_with_target_omega(angle, omega)
                };
                let reload_time = self.gun0.reload_time();
                reload_time <= 0.0 || turn_time > 0.8 * reload_time
            } else {
                false
            };
            let turning_for_missile = !is_reversing_orbit
                && !aiming_gun0
                && self.gun1.reload_time() <= 10.0 * TICK_LENGTH
                && missile_aim_info.is_some();
            if aiming_gun0 || turning_for_missile {
                want_to_boost = false;
            }

            // Quick turn towards the desired heading
            let target_omega = target_omega_opt.unwrap_or(0.0);
            quick_turn_with_target_omega(desired_heading, target_omega);

            // Boost Control
            if want_to_boost {
                let acc_angle = acc_cmd.angle();
                let is_aligned = angle_diff(heading(), acc_angle).abs() < 5.0f64.to_radians();
                if is_aligned {
                    activate_ability(Ability::Boost);
                } else {
                    deactivate_ability(Ability::Boost);
                }
            } else {
                deactivate_ability(Ability::Boost);
            }

            // --- Dedicated Missile Telemetry and Firing ---
            let mut current_missile_target = None;
            if let Some(c) = contacts.iter().find(|c| c.class == Class::Fighter) {
                current_missile_target =
                    Some((c.current_position(), c.current_velocity(), 0.0f32, c.class));
            } else if let Some(last_pos) = self.fighter_last_known_target_pos {
                let last_vel = self
                    .fighter_last_known_target_vel
                    .unwrap_or(Vec2::new(0.0, 0.0));
                current_missile_target = Some((last_pos, last_vel, 0.0f32, Class::Fighter));
            }

            if let Some((m_pos, m_vel, m_rssi, m_class)) = current_missile_target {
                let diff_to_desired = angle_diff(heading(), desired_heading);
                if self.gun1.reload_time() == 0.0 && diff_to_desired.abs() <= 15.0f64.to_radians() {
                    if let Some(sol) = &missile_solution {
                        if self.salvo_missiles_fired == 0 {
                            self.salvo_missiles_fired = 1;
                        } else {
                            self.salvo_missiles_fired =
                                (self.salvo_missiles_fired + 1) % SALVO_SIZE;
                        }
                        self.gun1.fire_at(sol);
                        self.ticks_since_missile_fired = 0;
                        debug!(
                            "Fighter {} fired salvo missile, state salvo_missiles_fired={}",
                            id(),
                            self.salvo_missiles_fired
                        );
                    }
                }

                let telemetry = TargetTelemetry {
                    position: m_pos,
                    velocity: m_vel,
                    rssi: m_rssi,
                    class: m_class,
                    tick: current_tick() as u8,
                };
                self.missile_radio.transmit(telemetry.serialize());
            }

            self.gun0.draw_debug();
            self.gun1.draw_debug();
        }
    }

    fn log_pd_aim_inputs(
        &self,
        pd_aim_info: Option<(f64, f64)>,
        desired_heading: f64,
        target_omega_opt: Option<f64>,
        best_pd_missile: &Option<(Contact, f64, Vec2, f64)>,
        us_kinematic: &KinematicState,
    ) {
        let Some((p_angle, p_omega)) = pd_aim_info else {
            return;
        };
        if desired_heading != p_angle || target_omega_opt != Some(p_omega) {
            return;
        }
        let Some((c, _, _, _)) = best_pd_missile else {
            return;
        };
    }

    fn compute_desired_heading(
        &self,
        pd_aim_info: Option<(f64, f64)>,
        gun_aim_info: Option<(f64, f64)>,
        missile_aim_info: Option<(f64, f64)>,
        acc_cmd: Vec2,
        is_reversing_orbit: bool,
    ) -> (f64, Option<f64>) {
        if is_reversing_orbit {
            // Point defense is a requirement and remains a priority
            if let Some((angle, omega)) = pd_aim_info {
                let turn_time = if max_angular_acceleration() < 1e-6 {
                    0.0
                } else {
                    quick_turn_time_with_target_omega(angle, omega)
                };
                let reload_time = self.gun0.reload_time();
                if reload_time <= 0.0 || turn_time > 0.8 * reload_time {
                    return (angle, Some(omega));
                }
            }
            // Otherwise, ignore missile firing angles and gun aiming, align with thrust to boost
            return (acc_cmd.angle(), None);
        }

        // 1. If we are aiming the gun0, the aim direction of the main gun and its omega.
        let gun0_solution = pd_aim_info.or(gun_aim_info);
        if let Some((angle, omega)) = gun0_solution {
            let turn_time = if max_angular_acceleration() < 1e-6 {
                0.0
            } else {
                quick_turn_time_with_target_omega(angle, omega)
            };
            let reload_time = self.gun0.reload_time();
            if reload_time <= 0.0 || turn_time > 0.8 * reload_time {
                return (angle, Some(omega));
            }
        }

        // 2. Otherwise, if a missile will be ready to fire in the next 10 ticks, the direction the missile should be fired.
        if self.gun1.reload_time() <= 10.0 * TICK_LENGTH {
            if let Some((angle, omega)) = missile_aim_info {
                return (angle, Some(omega));
            }
        }

        // 3. Otherwise, the direction in which we are thrusting this turn.
        (acc_cmd.angle(), None)
    }
}

#[cfg(test)]
mod fighter_duel_test {
    use super::*;
    use crate::physics::KinematicState;

    fn create_mock_missile(id: u32, pos: Vec2, vel: Vec2) -> Contact {
        Contact {
            id,
            kinematic: KinematicState::new(Class::Missile, pos, vel, Vec2::new(0.0, 0.0), 0),
            last_measurement_tick: 0,
            pos_uncertainty: 0.0,
            vel_uncertainty: 0.0,
            provisional: false,
            tracking_retry_count: 0,
            confirmation_attempts: 0,
            unscanned_in_range_ticks: 0,
            p_cov_x: [[0.0; 3]; 3],
            p_cov_y: [[0.0; 3]; 3],
            prioritize_scan: false,
            prev_scan_pos_uncertainty: None,
            low_improvement_consecutive_scans: 0,
            last_beam_width: None,
            last_beam_center: None,
            last_beam_center_pos: None,
            missile_scan_ticks_remaining: 0,
            scan_boundary_points: None,
            scan_boundary_vels: None,
        }
    }

    #[test]
    fn test_filter_missile_threats() {
        let our_pos = Vec2::new(0.0, 0.0);
        let our_vel = Vec2::new(0.0, 0.0);

        // 1. Missile moving away (closing <= 0) -> should be ignored
        let m_away = create_mock_missile(1, Vec2::new(100.0, 0.0), Vec2::new(10.0, 0.0));

        // 2. Missile heading straight at us, within 5s (dist = 400m, vel = -100m/s -> t_intercept = 4s) -> should be included
        let m_incoming = create_mock_missile(2, Vec2::new(400.0, 0.0), Vec2::new(-100.0, 0.0));

        // 3. Missile heading straight at us, but too far (dist = 1000m, vel = -100m/s -> t_intercept = 10s) -> should be ignored
        let m_far = create_mock_missile(3, Vec2::new(1000.0, 0.0), Vec2::new(-100.0, 0.0));

        // 4. Missile passing by (closest approach = 200m > 150m, pos = (300, 200), vel = (-100, 0)) -> should be ignored
        let m_miss = create_mock_missile(4, Vec2::new(300.0, 200.0), Vec2::new(-100.0, 0.0));

        // 5. Missile passing by very closely (closest approach = 50m <= 150m, pos = (300, 50), vel = (-100, 0)) -> should be included
        let m_close_pass = create_mock_missile(5, Vec2::new(300.0, 50.0), Vec2::new(-100.0, 0.0));

        // 6. Set of contacts
        let contacts = vec![m_away, m_incoming, m_far, m_miss, m_close_pass];
        let threats = filter_missile_threats(&contacts, our_pos, our_vel);

        // Should contain ID 5 (dist = 304.1m) and ID 2 (dist = 400m)
        assert_eq!(threats.len(), 2);
        assert_eq!(threats[0], 5); // Closest is 304.1m
        assert_eq!(threats[1], 2); // Farther is 400m
    }

    #[test]
    fn test_filter_missile_threats_limit_three() {
        let our_pos = Vec2::new(0.0, 0.0);
        let our_vel = Vec2::new(0.0, 0.0);

        // 4 missiles heading straight at us at different distances
        let m1 = create_mock_missile(1, Vec2::new(400.0, 0.0), Vec2::new(-100.0, 0.0));
        let m2 = create_mock_missile(2, Vec2::new(200.0, 0.0), Vec2::new(-100.0, 0.0));
        let m3 = create_mock_missile(3, Vec2::new(300.0, 0.0), Vec2::new(-100.0, 0.0));
        let m4 = create_mock_missile(4, Vec2::new(100.0, 0.0), Vec2::new(-100.0, 0.0));

        let contacts = vec![m1, m2, m3, m4];
        let threats = filter_missile_threats(&contacts, our_pos, our_vel);

        // Should return exactly the 3 closest: ID 4 (100m), ID 2 (200m), ID 3 (300m)
        assert_eq!(threats.len(), 3);
        assert_eq!(threats[0], 4);
        assert_eq!(threats[1], 2);
        assert_eq!(threats[2], 3);
    }

    #[test]
    fn test_can_missile_intercept() {
        let us = KinematicState::new(
            Class::Fighter,
            Vec2::new(0.0, 0.0),
            Vec2::new(0.0, 0.0),
            Vec2::new(0.0, 0.0),
            0,
        );

        // 1. Missile heading straight at us (should be interceptable)
        let m_incoming = KinematicState::new(
            Class::Missile,
            Vec2::new(400.0, 0.0),
            Vec2::new(-100.0, 0.0),
            Vec2::new(0.0, 0.0),
            0,
        );
        assert!(can_missile_intercept(&m_incoming, &us));

        // 2. Missile moving away (should not be interceptable)
        let m_away = KinematicState::new(
            Class::Missile,
            Vec2::new(100.0, 0.0),
            Vec2::new(50.0, 0.0),
            Vec2::new(0.0, 0.0),
            0,
        );
        assert!(!can_missile_intercept(&m_away, &us));

        // 3. Missile with very high perp velocity that cannot be cancelled (should not be interceptable)
        let m_skewed = KinematicState::new(
            Class::Missile,
            Vec2::new(100.0, 0.0),
            Vec2::new(-50.0, 500.0),
            Vec2::new(0.0, 0.0),
            0,
        );
        assert!(!can_missile_intercept(&m_skewed, &us));
    }

    thread_local! {
        static MOCK_RELOAD_0: std::cell::Cell<u32> = std::cell::Cell::new(0);
        static MOCK_RELOAD_1: std::cell::Cell<u32> = std::cell::Cell::new(0);
    }

    fn test_reload_ticks(id: usize) -> u32 {
        match id {
            0 => MOCK_RELOAD_0.with(|c| c.get()),
            1 => MOCK_RELOAD_1.with(|c| c.get()),
            _ => 0,
        }
    }

    #[test]
    fn test_compute_desired_heading() {
        let radio_manager = Rc::new(RefCell::new(RadioManager::new()));

        MOCK_RELOAD_0.with(|c| c.set(0));
        MOCK_RELOAD_1.with(|c| c.set(0));

        let mut ship = Ship {
            radar_controller: RadarController::new(),
            missile_guidance: MissileGuidance::new(),
            missile_radio: SecureRadio::new(1337, 0, radio_manager.clone()),
            fighter_radio: SecureRadio::new(1337, 4, radio_manager),
            fighter_target_id: None,
            fighter_msgs_received: 0,
            fighter_last_known_target_pos: None,
            fighter_last_known_target_vel: None,
            orbit_direction: 1.0,
            target_orbit_speed_fraction: 0.7,
            num_direction_changes: 0,
            pd_target_id: None,
            pd_bullets_fired: 0,
            gun0: FireControl::new(0, test_reload_ticks),
            gun1: FireControl::new(1, test_reload_ticks),
            force_reload: false,
            target_pos: Rc::new(RefCell::new(None)),
            ticks_since_reversal: 9999,
            salvo_missiles_fired: 0,
            salvo_aim_point: None,
            salvo_arrival_tick: 0,
            ticks_since_missile_fired: 9999,
        };

        let acc_cmd = Vec2::new(10.0, 20.0);

        // Case 1: Aiming gun0 (PD aim info takes precedence over normal gun aim info)
        let (heading, omega) =
            ship.compute_desired_heading(Some((1.0, 2.0)), Some((3.0, 4.0)), None, acc_cmd, false);
        assert_eq!(heading, 1.0);
        assert_eq!(omega, Some(2.0));

        // Case 2: Aiming gun0 (normal gun aim info takes precedence when PD is None)
        let (heading, omega) =
            ship.compute_desired_heading(None, Some((3.0, 4.0)), None, acc_cmd, false);
        assert_eq!(heading, 3.0);
        assert_eq!(omega, Some(4.0));

        // Case 3: We have no ammo in magazine (bullets_in_magazine = 0).
        // It should NOT aim gun0.
        // If missile is ready (reload_ticks(1) <= 10, which is 0 in tests), it aims missile.
        ship.gun0.bullets_in_magazine = 0;
        MOCK_RELOAD_0.with(|c| c.set(60));
        MOCK_RELOAD_1.with(|c| c.set(0));
        let (heading, omega) =
            ship.compute_desired_heading(None, Some((3.0, 4.0)), Some((5.0, 6.0)), acc_cmd, false);
        assert_eq!(heading, 5.0);
        assert_eq!(omega, Some(6.0));

        // Case 4: No ammo, and no missile aim info.
        // Should fall back to thrust direction (acc_cmd.angle()).
        MOCK_RELOAD_1.with(|c| c.set(300));
        let (heading, omega) =
            ship.compute_desired_heading(None, Some((3.0, 4.0)), None, acc_cmd, false);
        assert_eq!(heading, acc_cmd.angle());
        assert_eq!(omega, None);

        // Case 5: Reversing orbit, with PD active. Point defense must remain a priority.
        ship.gun0.bullets_in_magazine = 30;
        MOCK_RELOAD_0.with(|c| c.set(0));
        let (heading, omega) = ship.compute_desired_heading(
            Some((1.0, 2.0)),
            Some((3.0, 4.0)),
            Some((5.0, 6.0)),
            acc_cmd,
            true,
        );
        assert_eq!(heading, 1.0);
        assert_eq!(omega, Some(2.0));

        // Case 6: Reversing orbit, with PD None. Ignore missile firing angles and normal gun aiming.
        let (heading, omega) =
            ship.compute_desired_heading(None, Some((3.0, 4.0)), Some((5.0, 6.0)), acc_cmd, true);
        assert_eq!(heading, acc_cmd.angle());
        assert_eq!(omega, None);
    }
}
