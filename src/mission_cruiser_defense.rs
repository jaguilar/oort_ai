use oort_api::prelude::*;
use std::rc::Rc;
use std::cell::RefCell;
use crate::control::{quick_turn_with_target_omega, predict_lead, AngleTracker, MissileGuidance, TargetTelemetry};
use crate::radar::{RadarController, DefaultScanSliceGenerator, Contact};
use crate::radio::{SecureRadio, RadioManager};

fn cluster_contacts(contacts: &[Contact]) -> Vec<Vec<Contact>> {
    if contacts.is_empty() {
        return Vec::new();
    }
    if contacts.len() <= 3 {
        return contacts.iter().map(|c| vec![c.clone()]).collect();
    }
    let mut clusters: Vec<Vec<Contact>> = contacts.iter().map(|c| vec![c.clone()]).collect();
    while clusters.len() > 3 {
        let mut min_dist = f64::MAX;
        let mut merge_indices = (0, 0);
        for i in 0..clusters.len() {
            for j in (i + 1)..clusters.len() {
                for c1 in &clusters[i] {
                    for c2 in &clusters[j] {
                        let dist = c1.current_position().distance(c2.current_position());
                        if dist < min_dist {
                            min_dist = dist;
                            merge_indices = (i, j);
                        }
                    }
                }
            }
        }
        let (i, j) = merge_indices;
        let mut target_cluster = clusters[j].clone();
        clusters[i].append(&mut target_cluster);
        clusters.swap_remove(j);
    }
    clusters
}

struct ExpectedIntercept {
    position: Vec2,
    expiry_tick: u32,
}

pub struct Ship {
    radar_controller: RadarController,
    weapon_targets: [Option<u32>; 4], // Targets assigned to weapon slots 0, 1, and 2
    angle_tracker: AngleTracker,      // Tracks Gun 0 target angle rate for ship orientation
    intercept_plots: Vec<ExpectedIntercept>, // Intercept points to draw for active bullets
    missile_guidance: MissileGuidance,
    
    seen_target_ids: Vec<u32>,
    
    missile_radio: SecureRadio,

    // We don't send the missile's target information until the tick after we spawn it.
    // t0: Fire missile.
    // t1: Missile's first tick, tunes radio. We transmit.
    // t2: Missile receives telemetry.
    delay_missile_contact: Option<u32>,

    // NEW FIELDS FOR FIGHTER COORDINATION AND ORBIT BEHAVIOR
    fighter_radio: SecureRadio,
    fighter_targets_to_broadcast: Option<Vec<TargetTelemetry>>,
    fighter_broadcast_index: usize,
    fighter_target_id: Option<u32>,
    fighter_msgs_received: u32,
    fighter_last_known_target_pos: Option<Vec2>,
    fighter_last_known_target_vel: Option<Vec2>,

    // Orbit and movement fields
    orbit_direction: f64,
    current_period_ticks: u32,
    last_orbit_direction_change_tick: u32,
    num_direction_changes: u32,
}

fn predict_lead_exact(
    gun_pos: Vec2,
    our_vel: Vec2,
    bullet_speed: f64,
    target_pos: Vec2,
    target_vel: Vec2,
    target_accel: Vec2,
) -> Option<(f64, Vec2)> {
    let dp0 = target_pos - gun_pos;
    let r_len = dp0.length();
    if r_len < 1e-6 {
        return None;
    }
    let dv = target_vel - our_vel;
    let v_c = -dv.dot(dp0) / r_len;
    let t0 = r_len / (bullet_speed + v_c.max(0.0));

    let f = |t: f64| {
        let p_e = target_pos + t * target_vel + 0.5 * target_accel * t * (t + TICK_LENGTH);
        let d = p_e - gun_pos - t * our_vel;
        d.length() - bullet_speed * t
    };

    let df = |t: f64| {
        let p_e = target_pos + t * target_vel + 0.5 * target_accel * t * (t + TICK_LENGTH);
        let d = p_e - gun_pos - t * our_vel;
        let d_len = d.length();
        let d_prime = target_vel + target_accel * (t + 0.5 * TICK_LENGTH) - our_vel;
        if d_len > 1e-6 {
            d.dot(d_prime) / d_len - bullet_speed
        } else {
            -bullet_speed
        }
    };

    let clamp = |t: f64| {
        if t < 0.0 {
            0.0
        } else {
            t
        }
    };

    if let Some(t) = crate::control::newton_solve(t0, f, df, clamp, 20, 1e-4) {
        if t >= 0.0 {
            let p_e = target_pos + t * target_vel + 0.5 * target_accel * t * (t + TICK_LENGTH);
            let d = p_e - gun_pos - t * our_vel;
            if d.length() > 0.0 {
                return Some((t, d.normalize()));
            }
        }
    }
    None
}

impl Ship {
    pub fn new() -> Ship {
        let mut rc = RadarController::new();
        // Double the base scan range from 10000.0 to 20000.0
        rc.slice_generator = Box::new(DefaultScanSliceGenerator::new(0.6, 20000.0));

        let radio_manager = Rc::new(RefCell::new(RadioManager::new()));
        let missile_radio = SecureRadio::new(1337, 0, radio_manager.clone());
        let fighter_radio = SecureRadio::new(1338, 4, radio_manager);

        let mut mg = MissileGuidance::new();
        mg.target_channel = 3;
        mg.secure_radio = Some(missile_radio.clone());

        Ship {
            radar_controller: rc,
            weapon_targets: [None; 4],
            angle_tracker: AngleTracker::new(5.0),
            intercept_plots: Vec::new(),
            missile_guidance: mg,
            seen_target_ids: Vec::new(),
            missile_radio,
            delay_missile_contact: None,
            fighter_radio,
            fighter_targets_to_broadcast: None,
            fighter_broadcast_index: 0,
            fighter_target_id: None,
            fighter_msgs_received: 0,
            fighter_last_known_target_pos: None,
            fighter_last_known_target_vel: None,
            orbit_direction: if rand(0.0, 1.0) < 0.5 { 1.0 } else { -1.0 },
            current_period_ticks: ((rand(7.0, 13.0) / 2.0) / TICK_LENGTH).round() as u32,
            last_orbit_direction_change_tick: 0,
            num_direction_changes: 0,
        }
    }

    /// Selects a missile target from `fighters`. Contacts satisfying `in_priority_set` are
    /// preferred; if none qualify, a random contact from the full list is returned instead.
    fn select_missile_target<'a, F>(
        &self,
        fighters: &[&'a crate::radar::Contact],
        in_priority_set: F,
    ) -> Option<u32>
    where
        F: Fn(&crate::radar::Contact) -> bool,
    {
        let priority: Vec<&&crate::radar::Contact> = fighters.iter()
            .filter(|f| in_priority_set(f))
            .collect();

        let pool: Vec<&&crate::radar::Contact> = if !priority.is_empty() {
            priority
        } else {
            fighters.iter().collect()
        };
        
        // Draw green squares around all targets that were in the pool.
        for f in pool.iter() {
            draw_square(vec2(f.current_position().x as f64, f.current_position().y as f64), 300.0, rgb(0, 255, 0));
        }

        if pool.is_empty() {
            return None;
        }
        let idx = (rand(0.0, pool.len() as f64).floor() as usize).min(pool.len() - 1);
        Some(pool[idx].id)
    }

    fn fire_missile(&mut self, weapon_id: usize, target_id: u32) {
        fire(weapon_id);
        self.delay_missile_contact = Some(target_id);
    }

    fn num_guns_targeting(&self, target_id: u32, exclude_weapon: Option<usize>) -> usize {
        (0..3)
            .filter(|&w| Some(w) != exclude_weapon && self.weapon_targets[w] == Some(target_id))
            .count()
    }

    fn is_better_target(&self, w: usize, a: &crate::radar::Contact, b: &crate::radar::Contact) -> bool {
        let total_targeting_a = self.num_guns_targeting(a.id, Some(w));
        let total_targeting_b = self.num_guns_targeting(b.id, Some(w));

        let is_clean_a = total_targeting_a == 0;
        let is_clean_b = total_targeting_b == 0;

        if is_clean_a != is_clean_b {
            return is_clean_a;
        }

        if total_targeting_a != total_targeting_b {
            return total_targeting_a < total_targeting_b;
        }

        let ship_pos = position();
        match w {
            0 => {
                // Gun 0: Turreted heavy cannon. Targets closest fighter (same as frigate's turreted guns).
                let dist_a = ship_pos.distance(a.current_position());
                let dist_b = ship_pos.distance(b.current_position());
                dist_a < dist_b
            }
            1 => {
                // Gun 1: Left missile launcher. Prefers targets on the left side.
                let angle_a = (a.current_position() - ship_pos).angle();
                let angle_b = (b.current_position() - ship_pos).angle();
                let diff_a = angle_diff(heading(), angle_a);
                let diff_b = angle_diff(heading(), angle_b);
                let a_on_left = diff_a >= 0.0;
                let b_on_left = diff_b >= 0.0;
                if a_on_left != b_on_left {
                    a_on_left
                } else {
                    let dist_a = ship_pos.distance(a.current_position());
                    let dist_b = ship_pos.distance(b.current_position());
                    dist_a < dist_b
                }
            }
            2 => {
                // Gun 2: Right missile launcher. Prefers targets on the right side.
                let angle_a = (a.current_position() - ship_pos).angle();
                let angle_b = (b.current_position() - ship_pos).angle();
                let diff_a = angle_diff(heading(), angle_a);
                let diff_b = angle_diff(heading(), angle_b);
                let a_on_right = diff_a < 0.0;
                let b_on_right = diff_b < 0.0;
                if a_on_right != b_on_right {
                    a_on_right
                } else {
                    let dist_a = ship_pos.distance(a.current_position());
                    let dist_b = ship_pos.distance(b.current_position());
                    dist_a < dist_b
                }
            }
            _ => false,
        }
    }

    fn send_missile_contact(&mut self) {
        if let Some(cid) = self.delay_missile_contact {
            let contact = self.radar_controller.contacts().iter().find(|&c| c.id == cid);
            if let Some(contact) = contact {
                let telemetry = TargetTelemetry {
                    position: contact.current_position(),
                    velocity: contact.current_velocity(),
                    rssi: contact.rssi as f32,
                    class: contact.class,
                    tick: current_tick() as u8,
                };
                self.missile_radio.transmit(telemetry.serialize());
                debug!("Sent location of {} to missiles", contact.id);
            }
            self.delay_missile_contact = None;
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

            // 1. Update radar scheduler and contact database
            let mut priority_ids = Vec::new();
            if let Some(tid) = self.fighter_target_id {
                priority_ids.push(tid);
            }
            self.radar_controller.priority_targets = priority_ids;

            // 2. Receive and process fighter radio messages (always listen to ensure we get our assignment and pings)
            let target_idx = (id() as i32) - 2; // Fighter 2 -> 0, Fighter 3 -> 1, Fighter 4 -> 2
            let mut received_msg = None;
            if target_idx >= 0 && target_idx < 3 {
                let received = self.fighter_radio.receive();
                if let Some(payload) = received {
                    self.fighter_msgs_received += 1;
                    let telemetry = TargetTelemetry::deserialize(&payload);
                    debug!("Fighter {} received radio message #{}: pos={:?}, target_idx={}, target_idx+1={}", id(), self.fighter_msgs_received, telemetry.position, target_idx, target_idx + 1);
                    if self.fighter_msgs_received == (target_idx + 1) as u32 {
                        received_msg = Some(telemetry);
                    } else {
                        // Treat as a virtual radar ping to update our contact database
                        self.radar_controller.add_radio_ping(telemetry);
                    }
                }
            }

            self.radar_controller.update();
            self.send_missile_contact();

            // 3. Get contacts
            let contacts = self.radar_controller.contacts().to_vec();

            // 4. Update target tracking
            let received_assignment = self.fighter_msgs_received >= (target_idx + 1) as u32;
            if !received_assignment {
                self.fighter_radio.prepare_receive();
            }

            if received_assignment {
                
                if let Some(telemetry) = received_msg {
                    self.fighter_target_id = Some(self.radar_controller.add_radio_ping(telemetry));
                    debug!("Fighter {} acquired assigned target via radio at {:?}", id(), telemetry.position);
                }

                // If target is dead/None, select closest enemy fighter from radar contacts
                if self.fighter_target_id.is_none() || !self.radar_controller.has_contact(self.fighter_target_id.unwrap()) {
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
                        debug!("Fighter {} selected closest target: ID {}", id(), c.id);
                    }
                }

                // 4.5. Print target diagnostics
                if let Some(tid) = self.fighter_target_id {
                    debug!("Fighter {} tracking target ID: {}", id(), tid);
                    if let Some(contact) = self.radar_controller.get_contact(tid) {
                        let ticks_since_tracked = current_tick() - contact.last_scanned;
                        debug!("  Contact {} last scanned {} ticks ago (provisional={})", tid, ticks_since_tracked, contact.provisional);
                        debug!("  Position: {:?}", contact.current_position());
                        debug!("  Velocity: {:?}", contact.current_velocity());
                        // Draw a magenta circle at the contact position
                        draw_polygon(contact.current_position(), 50.0, 16, 0.0, rgb(255, 0, 255));
                    } else {
                        debug!("  Contact {} NOT found in radar database!", tid);
                    }
                } else {
                    debug!("Fighter {} tracking target ID: None", id());
                }

                // 4. Movement & Aiming/Firing
                if let Some(contact) = self.fighter_target_id.and_then(|tid| self.radar_controller.get_contact(tid)) {
                    let (target_pos, target_vel) = (contact.current_position(), contact.current_velocity());
                    let target_accel = contact.acceleration;
                    
                    // Orbit target at 3km
                    draw_line(position(), target_pos, rgb(0, 255, 255));

                    let r_orbit = 3000.0;
                    let rel_pos = position() - target_pos;
                    let rel_vel = velocity() - target_vel;
                    let d = rel_pos.length();
                    let u = if d > 1.0 { rel_pos / d } else { vec2(1.0, 0.0) };

                    let speed = rel_vel.length();
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
                        acc_cmd = 1.0 * (desired_vel - rel_vel);
                    } else if d > r_orbit + 200.0 {
                        // Outside: drive towards the circle on a tangent line
                        let phi = (r_orbit / d).asin();
                        let tangent_dir = (-u).rotate(-self.orbit_direction * phi);
                        let desired_vel = tangent_dir * target_speed;
                        acc_cmd = 1.0 * (desired_vel - rel_vel);
                    } else {
                        // On the circle: standard orbit control
                        let t = vec2(-u.y, u.x) * self.orbit_direction;
                        let centripetal_needed = speed.powi(2) / r_orbit;

                        // Radial control: regulate distance to r_orbit
                        let e_r = d - r_orbit;
                        let v_r = rel_vel.dot(u);
                        let kp_r = 0.5;
                        let kd_r = 1.0;
                        let radial_accel_mag = centripetal_needed + kp_r * e_r + kd_r * v_r;
                        
                        // Tangential control: regulate speed to target_speed
                        let v_t = rel_vel.dot(t);
                        let kp_t = 0.5;
                        let tangential_accel_mag = kp_t * (target_speed - v_t);

                        acc_cmd = -radial_accel_mag * u + tangential_accel_mag * t;
                    }

                    // Add feedforward target acceleration
                    acc_cmd += target_accel;

                    if acc_cmd.length() > max_acc {
                        acc_cmd = acc_cmd.normalize() * max_acc;
                    }

                    // Apply acceleration command for movement
                    accelerate(acc_cmd);

                    // 5. Aim and Shoot Cannon
                    let to_target = target_pos - position();
                    const BULLET_SPEED: f64 = 1000.0;
                    let mut target_angle_now = None;
                    if let Some((time_to_impact, lead_dir)) = predict_lead(
                        position(),
                        velocity(),
                        BULLET_SPEED,
                        target_pos,
                        target_vel,
                        target_accel,
                    ) {
                        let angle = lead_dir.angle();
                        target_angle_now = Some(angle);

                        // Draw line to predicted target position
                        let p_e = target_pos + time_to_impact * target_vel + 0.5 * target_accel * time_to_impact * (time_to_impact + TICK_LENGTH);
                        draw_line(position(), p_e, rgb(255, 255, 0));
                    }

                    let target_angle = if let Some(angle) = target_angle_now {
                        angle
                    } else {
                        to_target.angle()
                    };

                    let target_omega = self.angle_tracker.update(target_angle);
                    quick_turn_with_target_omega(target_angle, target_omega);

                    // Fire main gun (slot 0) if cooldown <= 5 and aligned
                    if let Some(angle_now) = target_angle_now {
                        let diff = angle_diff(heading(), angle_now);
                        if reload_ticks(0) <= 5 && diff.abs() < 0.15f64.to_radians() {
                            fire(0);
                        }
                    }
                } else {
                    // Hold station at geometric center / origin if no target
                    let center = vec2(0.0, 0.0);
                    let error = center - position();
                    accelerate(0.1 * error - 0.6 * velocity());
                }

                // --- Dedicated Missile Telemetry and Firing ---
                let mut current_missile_target = None;
                if let Some(c) = contacts.iter().find(|c| c.class == Class::Fighter) {
                    current_missile_target = Some((c.id, c.current_position(), c.current_velocity(), c.rssi as f32, c.class));
                } else if let Some(last_pos) = self.fighter_last_known_target_pos {
                    let last_vel = self.fighter_last_known_target_vel.unwrap_or(Vec2::new(0.0, 0.0));
                    let tid = self.fighter_target_id.unwrap_or(0);
                    current_missile_target = Some((tid, last_pos, last_vel, 0.0f32, Class::Fighter));
                }

                if let Some((tid, m_pos, _m_vel, _m_rssi, _m_class)) = current_missile_target {
                    if reload_ticks(1) == 0 {
                        self.fire_missile(1, tid);
                        debug!("Fighter {} fired missile at target {:?}", id(), m_pos);
                    }
                }
            }

            return;
        }

        let current_tick = current_tick();

        // 0. Update priority targets list based on weapon targeting and reload status
        let mut priority_ids = Vec::new();
        for w in 0..3 {
            if let Some(tid) = self.weapon_targets[w] {
                priority_ids.push(tid);
            }
        }
        self.radar_controller.priority_targets = priority_ids;
        self.radar_controller.update();
        self.send_missile_contact();

        // 2. Fetch active fighter contacts from the radar controller
        let contacts = self.radar_controller.contacts().to_vec();
        let fighters: Vec<crate::radar::Contact> = contacts.iter()
            .filter(|c| c.class != Class::Missile)
            .cloned()
            .collect();
        let fighter_refs: Vec<&crate::radar::Contact> = fighters.iter().collect();

        for f in &fighters {
            if !self.seen_target_ids.contains(&f.id) {
                self.seen_target_ids.push(f.id);
            }
        }

        // Cruiser target clustering and broadcasting
        if self.fighter_targets_to_broadcast.is_none() && self.fighter_broadcast_index == 0 {
            let active_fighters: Vec<Contact> = fighters.clone();
            let clusters = cluster_contacts(&active_fighters);
            
            if current_tick % 10 == 0 && !active_fighters.is_empty() {
                let cluster_sizes: Vec<usize> = clusters.iter().map(|c| c.len()).collect();
                debug!("Cruiser clustering check: active_fighters count={}, clusters count={}, cluster_sizes={:?}", active_fighters.len(), clusters.len(), cluster_sizes);
            }

            if clusters.len() == 3 && clusters[0].len() >= 2 && clusters[1].len() >= 2 && clusters[2].len() >= 2 {
                let t0 = TargetTelemetry {
                    position: clusters[0][0].current_position(),
                    velocity: clusters[0][0].current_velocity(),
                    rssi: clusters[0][0].rssi as f32,
                    class: clusters[0][0].class,
                    tick: current_tick as u8,
                };
                let t1 = TargetTelemetry {
                    position: clusters[1][0].current_position(),
                    velocity: clusters[1][0].current_velocity(),
                    rssi: clusters[1][0].rssi as f32,
                    class: clusters[1][0].class,
                    tick: current_tick as u8,
                };
                let t2 = TargetTelemetry {
                    position: clusters[2][0].current_position(),
                    velocity: clusters[2][0].current_velocity(),
                    rssi: clusters[2][0].rssi as f32,
                    class: clusters[2][0].class,
                    tick: current_tick as u8,
                };
                self.fighter_targets_to_broadcast = Some(vec![t0, t1, t2]);
                self.fighter_broadcast_index = 0;
                debug!("Cruiser detected 3 distinct enemy clusters! Staging broadcasts.");
            }
        }

        if let Some(ref targets) = self.fighter_targets_to_broadcast {
            if self.fighter_broadcast_index < 3 {
                let telemetry = targets[self.fighter_broadcast_index];
                self.fighter_radio.transmit(telemetry.serialize());
                let ch = self.fighter_radio.channel_offset;
                debug!("Cruiser broadcast target {} on fighter channel (slot 1, offset {}): pos={:?}, vel={:?}, tick={}", self.fighter_broadcast_index, ch, telemetry.position, telemetry.velocity, telemetry.tick);
                self.fighter_broadcast_index += 1;
            }
            if self.fighter_broadcast_index == 3 {
                self.fighter_targets_to_broadcast = None;
            }
        }

        // 3. Update Weapon Assignments
        // Prune targets that are no longer tracked/alive
        for w in 0..3 {
            if let Some(tid) = self.weapon_targets[w] {
                if !fighters.iter().any(|f| f.id == tid) {
                    self.weapon_targets[w] = None;
                }
            }
        }

        // Build list of weapons that are ready and need assignment (currently have None target)
        let mut assignment_order = Vec::new();
        if self.weapon_targets[0].is_none() {
            assignment_order.push(0);
        }
        if self.weapon_targets[1].is_none() && reload_ticks(1) <= 4 {
            assignment_order.push(1);
        }
        if self.weapon_targets[2].is_none() && reload_ticks(2) <= 4 {
            assignment_order.push(2);
        }

        for &w in &assignment_order {
            if w == 0 {
                let mut best_fighter: Option<&crate::radar::Contact> = None;
                for f in &fighters {
                    // If we've seen less than 3 targets, do not allow duplicate target assignments.
                    if self.seen_target_ids.len() < 3 {
                        let total_targeting = self.num_guns_targeting(f.id, Some(w));
                        if total_targeting > 0 {
                            continue;
                        }
                    }

                    if let Some(best) = best_fighter {
                        if self.is_better_target(w, f, best) {
                            best_fighter = Some(f);
                        }
                    } else {
                        best_fighter = Some(f);
                    }
                }
                if let Some(f) = best_fighter {
                    self.weapon_targets[w] = Some(f.id);
                }
            } else {
                
            }
        }

        let is_near_angle = |f: &crate::radar::Contact, angle: f64| {
            let dir = f.current_position() - position();
            angle_diff(angle, dir.angle()).abs() <= 67.5f64.to_radians()
        };
        if reload_ticks(1) == 0 {
            let left_angle = heading() + std::f64::consts::FRAC_PI_2;
            debug!("considering left missile targets");
            self.weapon_targets[1] = self.select_missile_target(&fighter_refs, |f| is_near_angle(f, left_angle));
        } else if reload_ticks(2) == 0 {
            debug!("considering right missile targets");
            let right_angle = heading() - std::f64::consts::FRAC_PI_2;
            self.weapon_targets[2] = self.select_missile_target(&fighter_refs, |f| is_near_angle(f, right_angle));
        }

        // 4. Weapon Aiming and Firing
        // Gun 0: Turreted heavy cannon (Bullet Speed: 2000.0 m/s)
        if let Some(tid) = self.weapon_targets[0] {
            if let Some(f) = fighters.iter().find(|fighter| fighter.id == tid) {
                const GUN0_BULLET_SPEED: f64 = 2000.0;
                let gun0_pos = position();

                if let Some((time_to_impact, lead_dir)) = predict_lead_exact(
                    gun0_pos,
                    velocity(),
                    GUN0_BULLET_SPEED,
                    f.current_position(),
                    f.current_velocity(),
                    f.acceleration,
                ) {
                    let lead_angle = lead_dir.angle();
                    aim(0, lead_angle);

                    // Rotate the ship to face the target generally to facilitate radar sweep/movement
                    let target_omega = self.angle_tracker.update(lead_angle);
                    quick_turn_with_target_omega(lead_angle, target_omega);

                    // Visualization
                    let p_e = f.position_at(current_tick + (time_to_impact / TICK_LENGTH).round() as u32);
                    draw_line(gun0_pos, p_e, rgb(255, 0, 0));
                    draw_square(p_e, 25.0, rgb(255, 0, 0));

                    if reload_ticks(0) == 0 {
                        fire(0);
                        let expiry_tick = current_tick + (time_to_impact / TICK_LENGTH).round() as u32;
                        self.intercept_plots.push(ExpectedIntercept {
                            position: p_e,
                            expiry_tick,
                        });
                    }
                } else {
                    let direct_angle = (f.current_position() - gun0_pos).angle();
                    aim(0, direct_angle);
                    let target_omega = self.angle_tracker.update(direct_angle);
                    quick_turn_with_target_omega(direct_angle, target_omega);
                }
            }
        } else {
            // Turn ship towards fallback target
            let fallback_target = self.weapon_targets[1]
                .or(self.weapon_targets[2])
                .and_then(|tid| fighters.iter().find(|f| f.id == tid));
            if let Some(f) = fallback_target {
                let direct_angle = (f.current_position() - position()).angle();
                let target_omega = self.angle_tracker.update(direct_angle);
                quick_turn_with_target_omega(direct_angle, target_omega);
            }
        }

        if self.missile_radio.avail() {
            let mut launched_slot = None;
            if self.weapon_targets[1].is_some() && reload_ticks(1) == 0 {
                launched_slot = Some(1);
            } else if self.weapon_targets[2].is_some() && reload_ticks(2) == 0 {
                launched_slot = Some(2);
            }

            if let Some(w) = launched_slot {
                if let Some(tid) = self.weapon_targets[w] {
                    self.fire_missile(w, tid);
                    if let Some(f) = fighters.iter().find(|fighter| fighter.id == tid) {
                        draw_diamond(f.current_position(), 300.0, rgb(78, 188, 44));
                    }
                    
                    // Clear assignment immediately to target a different enemy on subsequent launch
                    self.weapon_targets[w] = None;
                }
            }
        }

        // 5. Draw expected intercept plots for debug
        self.intercept_plots.retain(|plot| current_tick <= plot.expiry_tick);
        for plot in &self.intercept_plots {
            draw_polygon(plot.position, 8.0, 8, 0.0, rgb(255, 0, 0));
        }

        // 6. Draw a blue triangle at each contact's estimated position
        for contact in contacts {
            draw_triangle(contact.current_position(), 15.0, rgb(0, 0, 255));
        }
    }
}
