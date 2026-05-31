use oort_api::prelude::*;
use crate::control::{quick_turn_with_target_omega, predict_lead, AngleTracker, MissileGuidance, TargetTelemetry, TargetTracker};
use crate::radar::{RadarController, DefaultScanSliceGenerator, Contact};
use crate::radio::SecureRadio;

pub struct Ship {
    radar_controller: RadarController,
    angle_tracker: AngleTracker,      // Tracks Gun 0 target angle rate for ship orientation
    missile_guidance: MissileGuidance,
    
    missile_radio: SecureRadio,
    fighter_radio: SecureRadio,
    fighter_target: Option<TargetTracker>,
    fighter_target_id: Option<u32>,
    fighter_msgs_received: u32,
    fighter_last_known_target_pos: Option<Vec2>,
    fighter_last_known_target_vel: Option<Vec2>,
    
    // Orbit and movement fields
    orbit_direction: f64,
    current_period_ticks: u32,
    last_orbit_direction_change_tick: u32,
    num_direction_changes: u32,

    // Point Defense tracking state
    pd_target_id: Option<u32>,
    pd_bullets_fired: u32,

    // Magazine reload tracking state
    bullets_in_magazine: u32,
    last_reload_ticks: u32,
    force_reload: bool,
}

impl Ship {
    pub fn new() -> Ship {
        let mut rc = RadarController::new();
        // Double the base scan range from 10000.0 to 20000.0
        rc.slice_generator = Box::new(DefaultScanSliceGenerator::new(0.6, 20000.0));

        let missile_radio = SecureRadio::new(1337, 0);
        let fighter_radio = SecureRadio::new(1337, 4);

        let mut mg = MissileGuidance::new();
        mg.target_channel = 3;
        mg.secure_radio = Some(missile_radio);
        mg.fuel_economy_dv_threshold = 1000.0f64;

        // Pre-tune the radio slot 0 for tick 0 to avoid the 1-tick delay
        if class() == Class::Fighter {
            fighter_radio.prepare_receive(0);
        } else {
            missile_radio.prepare_receive(0);
        }

        Ship {
            radar_controller: rc,
            angle_tracker: AngleTracker::new(5.0),
            missile_guidance: mg,
            missile_radio,
            fighter_radio,
            fighter_target: None,
            fighter_target_id: None,
            fighter_msgs_received: 0,
            fighter_last_known_target_pos: None,
            fighter_last_known_target_vel: None,
            orbit_direction: if rand(0.0, 1.0) < 0.5 { 1.0 } else { -1.0 },
            current_period_ticks: ((rand(7.0, 13.0) / 2.0) / TICK_LENGTH).round() as u32,
            last_orbit_direction_change_tick: 0,
            num_direction_changes: 0,
            pd_target_id: None,
            pd_bullets_fired: 0,
            bullets_in_magazine: 30,
            last_reload_ticks: 0,
            force_reload: false,
        }
    }

    pub fn tick(&mut self) {
        if class() == Class::Missile {
            self.missile_guidance.tick();
            return;
        }

        if class() == Class::Fighter {
            debug!("Fighter ID: {}", id());
            debug!("Position: {:?}", position());

            // --- Magazine Tracking ---
            let current_reload = reload_ticks(0);
            if current_reload > self.last_reload_ticks {
                self.bullets_in_magazine = self.bullets_in_magazine.saturating_sub(1);
            }
            self.last_reload_ticks = current_reload;

            if self.bullets_in_magazine == 0 && current_reload == 0 {
                self.bullets_in_magazine = 30;
            }
            debug!("Magazine bullets: {}", self.bullets_in_magazine);

            // 0. Update priority targets: main enemy and closing missiles
            let mut priority_ids = Vec::new();
            if let Some(fid) = self.fighter_target_id {
                priority_ids.push(fid);
            }
            for c in self.radar_controller.contacts().iter().filter(|c| c.class == Class::Missile) {
                let r = c.current_position() - position();
                let dist = r.length();
                if dist > 0.0 {
                    let v_rel = c.current_velocity() - velocity();
                    let closing = -r.dot(v_rel) / dist;
                    if closing > 0.0 {
                        let t_intercept = dist / closing;
                        if t_intercept <= 3.0 && t_intercept >= 0.75 {
                            priority_ids.push(c.id);
                        }
                    }
                }
            }
            self.radar_controller.priority_targets = priority_ids;

            // 1. Update radar scheduler and contact database
            self.radar_controller.update();

            let contacts = self.radar_controller.contacts();
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
                            incoming_missile_next_5s = true;
                            break;
                        }
                    }
                }
            }

            self.force_reload = self.bullets_in_magazine < 15 && self.bullets_in_magazine > 0 && !incoming_missile_next_5s;
            if self.force_reload {
                debug!("FORCE RELOAD ACTIVE: Firing remaining {} bullets to reload", self.bullets_in_magazine);
            }

            // 2. Receive and process fighter radio messages (only if we don't have a target yet)
            if self.fighter_target.is_none() {
                let received = self.fighter_radio.receive();
                self.fighter_radio.prepare_receive(0);

                if let Some(payload) = received {
                    self.fighter_msgs_received += 1;
                    let target_idx = (id() as i32) - 2; // Fighter 2 -> 0, Fighter 3 -> 1, Fighter 4 -> 2
                    if target_idx >= 0 && target_idx < 3 && self.fighter_msgs_received == (target_idx + 1) as u32 {
                        let telemetry = TargetTelemetry::deserialize(&payload);
                        let mut tracker = TargetTracker::new();
                        tracker.update(current_tick(), telemetry.position, telemetry.velocity);
                        self.fighter_target = Some(tracker);
                        self.fighter_target_id = None;
                        self.fighter_last_known_target_pos = Some(telemetry.position);
                        self.fighter_last_known_target_vel = Some(telemetry.velocity);
                        debug!("Fighter {} acquired initial target via radio at {:?}", id(), telemetry.position);
                    }
                }
            }

            // 3. Update target tracking
            let mut target_contact = None;

            if let Some(ref mut tracker) = self.fighter_target {
                if let Some(tid) = self.fighter_target_id {
                    target_contact = contacts.iter().find(|c| c.id == tid && c.class == Class::Fighter).cloned();
                } else {
                    let (pred_pos, _) = tracker.extrapolate(current_tick());
                    let mut best_c = None;
                    let mut min_d = 2000.0;
                    for c in contacts.iter().filter(|c| c.class == Class::Fighter) {
                        let d = c.current_position().distance(pred_pos);
                        if d < min_d {
                            min_d = d;
                            best_c = Some(c.clone());
                        }
                    }
                    if let Some(c) = best_c {
                        let cid = c.id;
                        self.fighter_target_id = Some(cid);
                        target_contact = Some(c);
                        debug!("Fighter {} matched target to radar contact ID {}", id(), cid);
                    }
                }

                if let Some(ref c) = target_contact {
                    tracker.update(current_tick(), c.current_position(), c.current_velocity());
                    self.fighter_last_known_target_pos = Some(c.current_position());
                    self.fighter_last_known_target_vel = Some(c.current_velocity());
                } else {
                    // Target has disappeared. If we previously had it matched, it means it is killed!
                    if self.fighter_target_id.is_some() {
                        self.fighter_target = None;
                        self.fighter_target_id = None;
                        debug!("Fighter {} target was killed or lost", id());
                    }
                }
            }

            // If we don't have a target, try to find one from the radar contacts!
            if self.fighter_target.is_none() {
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
                    let mut tracker = TargetTracker::new();
                    tracker.update(current_tick(), c.current_position(), c.current_velocity());
                    self.fighter_target = Some(tracker);
                    self.fighter_target_id = Some(c.id);
                    self.fighter_last_known_target_pos = Some(c.current_position());
                    self.fighter_last_known_target_vel = Some(c.current_velocity());
                    debug!("Fighter {} acquired target ID {} via radar", id(), c.id);
                }
            }

            // If target is killed/lost, prioritize closest enemy fighter to the target's last known position
            if self.fighter_target.is_none() {
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
                        let mut tracker = TargetTracker::new();
                        tracker.update(current_tick(), c.current_position(), c.current_velocity());
                        self.fighter_target = Some(tracker);
                        self.fighter_target_id = Some(c.id);
                        self.fighter_last_known_target_pos = Some(c.current_position());
                        self.fighter_last_known_target_vel = Some(c.current_velocity());
                        debug!("Fighter {} selected next closest target: ID {}", id(), c.id);
                    }
                }
            }

            // Draw a line from our ship through the origin
            draw_line(position(), -position(), rgb(0, 255, 255));

            let r_orbit = 15000.0;
            let pos = position();
            let vel = velocity();
            let d = pos.length();
            let u = if d > 1.0 { pos / d } else { vec2(1.0, 0.0) };

            let speed = vel.length();
            let max_acc = max_forward_acceleration();

            let margin = 0.90; // Reserve 10% of acceleration for radial/tangential control
            let target_speed = (r_orbit * max_acc * margin).sqrt();

            // Periodic orbit direction change (randomly between 7.0 and 13.0 seconds)
            let ticks_since_change = current_tick() - self.last_orbit_direction_change_tick;
            if ticks_since_change >= self.current_period_ticks {
                self.orbit_direction = -self.orbit_direction;
                let t_seconds = rand(13.0, 17.0);
                self.current_period_ticks = (t_seconds / TICK_LENGTH).round() as u32;
                self.last_orbit_direction_change_tick = current_tick();
                self.num_direction_changes += 1;
                debug!("Orbit direction flipped to {}. Next change in {:.1}s", self.orbit_direction, t_seconds);
            }

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
                let t = vec2(-u.y, u.x) * self.orbit_direction;
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
            let is_boosting = active_abilities().get_ability(Ability::Boost);
            let mut want_to_boost = false;
            
            if speed < target_speed - 100.0 && reload_ticks(1) > 20 {
                want_to_boost = true;
            }

            // 5. Point Defense and Gunnery/Missile Aiming
            let mut desired_heading = heading();
            const BULLET_SPEED: f64 = 1000.0;
            let mut target_angle_now = None;

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
                        if t_intercept <= 3.0 && t_intercept >= 0.75 {
                            if let Some((_time_to_impact, lead_dir)) = predict_lead(
                                position(),
                                velocity(),
                                BULLET_SPEED,
                                c.current_position(),
                                c.current_velocity(),
                                c.acceleration,
                            ) {
                                pd_candidates.push((c.clone(), t_intercept, lead_dir));
                            }
                        }
                    }
                }
            }

            // Target selection / switching logic: switch targets after 5 bullets fired
            let mut best_pd_missile: Option<(Contact, f64, Vec2)> = None;
            if !pd_candidates.is_empty() {
                let current_idx = self.pd_target_id.and_then(|id| {
                    pd_candidates.iter().position(|(c, _, _)| c.id == id)
                });

                if let Some(idx) = current_idx {
                    if self.pd_bullets_fired >= 5 {
                        let other_candidates: Vec<&(Contact, f64, Vec2)> = pd_candidates.iter()
                            .enumerate()
                            .filter(|&(i, _)| i != idx)
                            .map(|(_, item)| item)
                            .collect();

                        if !other_candidates.is_empty() {
                            let best_other = other_candidates.iter()
                                .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
                                .unwrap();
                            best_pd_missile = Some((*best_other).clone());
                            self.pd_target_id = Some(best_other.0.id);
                            self.pd_bullets_fired = 0;
                            debug!("PD: Switched to next missile ID {} after 5 shots", best_other.0.id);
                        } else {
                            let current = &pd_candidates[idx];
                            best_pd_missile = Some(current.clone());
                        }
                    } else {
                        let current = &pd_candidates[idx];
                        best_pd_missile = Some(current.clone());
                    }
                } else {
                    let best = pd_candidates.iter()
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

            let mut pd_aim_angle = None;
            if let Some((ref c, _, lead_dir)) = best_pd_missile {
                pd_aim_angle = Some(lead_dir.angle());
                debug!("POINT DEFENSE: intercepting incoming missile ID {} in {:.2}s (bullets fired: {})", c.id, best_pd_missile.as_ref().unwrap().1, self.pd_bullets_fired);
            }

            if let Some(ref tracker) = self.fighter_target {
                let (target_pos, target_vel) = tracker.extrapolate(current_tick());
                
                // Calculate target angle for cannon
                if let Some((time_to_impact, lead_dir)) = predict_lead(
                    position(),
                    velocity(),
                    BULLET_SPEED,
                    target_pos,
                    target_vel,
                    tracker.acceleration(),
                ) {
                    let angle = lead_dir.angle();
                    target_angle_now = Some(angle);

                    // Draw line to predicted target position (only if we're not aiming at a point-defense threat)
                    if pd_aim_angle.is_none() {
                        let p_e = target_pos + time_to_impact * target_vel + 0.5 * tracker.acceleration() * time_to_impact * (time_to_impact + TICK_LENGTH);
                        draw_line(position(), p_e, rgb(255, 255, 0));
                    }
                }

                let cannon_aim_angle = if let Some(angle) = target_angle_now {
                    angle
                } else {
                    (target_pos - position()).angle()
                };

                // Fire main gun (slot 0)
                if self.force_reload {
                    fire(0);
                } else if let Some(pd_angle) = pd_aim_angle {
                    let diff = angle_diff(heading(), pd_angle);
                    if reload_ticks(0) == 0 && diff.abs() < 2.0f64.to_radians() {
                        fire(0);
                        self.pd_bullets_fired += 1;
                    }
                } else {
                    // Normal gun firing logic against enemy ship
                    if let Some(angle_now) = target_angle_now {
                        let diff = angle_diff(heading(), angle_now);
                        if reload_ticks(0) <= 5 && diff.abs() < 0.15f64.to_radians() {
                            fire(0);
                        }
                    }
                }

                // Missile Firing logic
                let mut turn_for_missile = false;
                let mut missile_aim_angle = 0.0;
                
                if reload_ticks(1) <= 20 && !is_boosting {
                    let to_target = target_pos - position();
                    if to_target.length() > 0.0 {
                        let d_hat = to_target.normalize();
                        let d_perp = vec2(-d_hat.y, d_hat.x);
                        let v_perp = velocity().dot(d_perp);
                        let alpha = if v_perp.abs() <= 100.0 {
                            (-v_perp / 100.0).asin()
                        } else {
                            -v_perp.signum() * std::f64::consts::FRAC_PI_2
                        };
                        missile_aim_angle = d_hat.angle() + alpha;
                        turn_for_missile = true;
                    }
                }

                // Select desired heading: PD takes absolute priority
                if let Some(pd_angle) = pd_aim_angle {
                    desired_heading = pd_angle;
                    want_to_boost = false;
                } else if turn_for_missile {
                    desired_heading = missile_aim_angle;
                    want_to_boost = false; // Disable boost while doing missile alignment
                } else if want_to_boost {
                    desired_heading = acc_cmd.angle();
                } else {
                    desired_heading = cannon_aim_angle;
                }
            } else {
                // If we don't have an active fighter target but there's a PD threat, turn towards it
                if self.force_reload {
                    fire(0);
                } else if let Some(pd_angle) = pd_aim_angle {
                    desired_heading = pd_angle;
                    let diff = angle_diff(heading(), pd_angle);
                    if reload_ticks(0) == 0 && diff.abs() < 2.0f64.to_radians() {
                        fire(0);
                        self.pd_bullets_fired += 1;
                    }
                }
            }

            // Quick turn towards the desired heading
            let target_omega = self.angle_tracker.update(desired_heading);
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
                current_missile_target = Some((c.current_position(), c.current_velocity()));
            } else if let Some(last_pos) = self.fighter_last_known_target_pos {
                let last_vel = self.fighter_last_known_target_vel.unwrap_or(Vec2::new(0.0, 0.0));
                current_missile_target = Some((last_pos, last_vel));
            }

            if let Some((m_pos, m_vel)) = current_missile_target {
                let telemetry = TargetTelemetry {
                    position: m_pos,
                    velocity: m_vel,
                };
                self.missile_radio.transmit(0, telemetry.serialize());

                if reload_ticks(1) == 0 {
                    fire(1);
                    debug!("Fighter {} fired missile at target {:?}", id(), m_pos);
                }
            }
        }
    }
}
