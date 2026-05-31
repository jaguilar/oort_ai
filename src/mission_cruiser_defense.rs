use oort_api::prelude::*;
use crate::control::{quick_turn_with_target_omega, predict_lead, AngleTracker, MissileGuidance, TargetTelemetry, TargetTracker};
use crate::radar::{RadarController, DefaultScanSliceGenerator, Contact};
use crate::radio::SecureRadio;

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
    
    // Missile launching control state
    last_missile_fired_tick: u32,
    broadcast_target_id: Option<u32>,
    broadcast_ticks: u32,
    
    missile_launch_history: Vec<(u32, u32)>, // Tracks targets recently fired at by missiles: (target_id, tick_launched)
    seen_target_ids: Vec<u32>,
    missile_radio: SecureRadio,

    // NEW FIELDS FOR FIGHTER COORDINATION AND ORBIT BEHAVIOR
    fighter_radio: SecureRadio,
    fighter_targets_to_broadcast: Option<Vec<TargetTelemetry>>,
    fighter_broadcast_index: usize,
    fighter_target: Option<TargetTracker>,
    fighter_target_id: Option<u32>,
    fighter_msgs_received: u32,
    fighter_last_known_target_pos: Option<Vec2>,
    fighter_last_known_target_vel: Option<Vec2>,
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

        let missile_radio = SecureRadio::new(1337, 0);
        let fighter_radio = SecureRadio::new(1337, 4);

        let mut mg = MissileGuidance::new();
        mg.target_channel = 3;
        mg.secure_radio = Some(missile_radio);

        // Pre-tune the radio slot 0 for tick 0 to avoid the 1-tick delay
        if class() == Class::Fighter {
            fighter_radio.prepare_receive(0);
        } else {
            missile_radio.prepare_receive(0);
        }

        Ship {
            radar_controller: rc,
            weapon_targets: [None; 4],
            angle_tracker: AngleTracker::new(5.0),
            intercept_plots: Vec::new(),
            missile_guidance: mg,
            last_missile_fired_tick: 0,
            broadcast_target_id: None,
            broadcast_ticks: 0,
            missile_launch_history: Vec::new(),
            seen_target_ids: Vec::new(),
            missile_radio,
            fighter_radio,
            fighter_targets_to_broadcast: None,
            fighter_broadcast_index: 0,
            fighter_target: None,
            fighter_target_id: None,
            fighter_msgs_received: 0,
            fighter_last_known_target_pos: None,
            fighter_last_known_target_vel: None,
        }
    }

    fn num_guns_targeting(&self, target_id: u32, exclude_weapon: Option<usize>) -> usize {
        let active_guns = (0..3)
            .filter(|&w| Some(w) != exclude_weapon && self.weapon_targets[w] == Some(target_id))
            .count();
        let recent_missiles = self.missile_launch_history.iter().filter(|&&(id, _)| id == target_id).count();
        active_guns + recent_missiles
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

    pub fn tick(&mut self) {
        if class() == Class::Missile {
            self.missile_guidance.tick();
            return;
        }

        if class() == Class::Fighter {
            debug!("Fighter ID: {}", id());
            debug!("Position: {:?}", position());

            // 1. Update radar scheduler and contact database
            self.radar_controller.update();

            let contacts = self.radar_controller.contacts();
            for c in contacts.iter().filter(|c| c.class == Class::Fighter) {
                self.fighter_last_known_target_pos = Some(c.current_position());
                self.fighter_last_known_target_vel = Some(c.current_velocity());
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

            // 4. Movement: orbit target at 2000m
            if let Some(ref tracker) = self.fighter_target {
                let (target_pos, target_vel) = tracker.extrapolate(current_tick());
                let to_target = target_pos - position();
                let dist = to_target.length();
                
                if dist > 1.0 {
                    let target_dir = to_target.normalize();
                    let orbit_radius = 2000.0;
                    let angle = if dist > orbit_radius {
                        (orbit_radius / dist).asin()
                    } else {
                        std::f64::consts::FRAC_PI_2
                    };

                    // Orbit counterclockwise
                    let cos_a = angle.cos();
                    let sin_a = angle.sin();
                    let desired_vel_dir = vec2(
                        target_dir.x * cos_a - target_dir.y * sin_a,
                        target_dir.x * sin_a + target_dir.y * cos_a,
                    );

                    let desired_speed = 400.0;
                    let desired_vel = target_vel + desired_vel_dir * desired_speed;
                    let speed_err = desired_vel - velocity();
                    accelerate(0.5 * speed_err);
                } else {
                    accelerate(target_vel - velocity());
                }

                // 5. Aim and Shoot Cannon
                const BULLET_SPEED: f64 = 1000.0;
                let mut target_angle_now = None;
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

                    // Draw line to predicted target position
                    let p_e = target_pos + time_to_impact * target_vel + 0.5 * tracker.acceleration() * time_to_impact * (time_to_impact + TICK_LENGTH);
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

        // 1. Update radar scheduler and contact database
        self.radar_controller.update();

        // 2. Fetch active fighter contacts from the radar controller
        let contacts = self.radar_controller.contacts();
        let fighters: Vec<&crate::radar::Contact> = contacts.iter()
            .filter(|c| c.class == Class::Fighter)
            .collect();

        for f in &fighters {
            if !self.seen_target_ids.contains(&f.id) {
                self.seen_target_ids.push(f.id);
            }
        }

        // Cruiser target clustering and broadcasting
        if self.fighter_targets_to_broadcast.is_none() && self.fighter_broadcast_index == 0 {
            let active_fighters: Vec<Contact> = fighters.iter().map(|&f| f.clone()).collect();
            let clusters = cluster_contacts(&active_fighters);
            if clusters.len() == 3 && clusters[0].len() >= 2 && clusters[1].len() >= 2 && clusters[2].len() >= 2 {
                let t0 = TargetTelemetry {
                    position: clusters[0][0].current_position(),
                    velocity: clusters[0][0].current_velocity(),
                };
                let t1 = TargetTelemetry {
                    position: clusters[1][0].current_position(),
                    velocity: clusters[1][0].current_velocity(),
                };
                let t2 = TargetTelemetry {
                    position: clusters[2][0].current_position(),
                    velocity: clusters[2][0].current_velocity(),
                };
                self.fighter_targets_to_broadcast = Some(vec![t0, t1, t2]);
                self.fighter_broadcast_index = 0;
                debug!("Cruiser detected 3 distinct enemy clusters! Staging broadcasts.");
            }
        }

        if let Some(ref targets) = self.fighter_targets_to_broadcast {
            if self.fighter_broadcast_index < 3 {
                let telemetry = targets[self.fighter_broadcast_index];
                self.fighter_radio.transmit(1, telemetry.serialize());
                debug!("Cruiser broadcast target {} on fighter channel (slot 1)", self.fighter_broadcast_index);
                self.fighter_broadcast_index += 1;
            }
            if self.fighter_broadcast_index == 3 {
                self.fighter_targets_to_broadcast = None;
            }
        }

        // Clean up missile launch history, keeping only entries within the last 4.0 seconds (4.0 / TICK_LENGTH ticks)
        let ticks_limit = (4.0 / TICK_LENGTH).round() as u32;
        self.missile_launch_history.retain(|&(_, tick)| current_tick - tick <= ticks_limit);

        // 3. Update Weapon Assignments
        // Prune targets that are no longer tracked/alive
        for w in 0..3 {
            if let Some(tid) = self.weapon_targets[w] {
                if !fighters.iter().any(|f| f.id == tid) {
                    self.weapon_targets[w] = None;
                }
            }
        }

        // Missile exception for Guns 1 and 2: if target was recently launched at,
        // and other unlaunched targets exist, clear target to find a new one.
        for &w in &[1, 2] {
            if let Some(tid) = self.weapon_targets[w] {
                let target_in_history = self.missile_launch_history.iter().any(|&(id, _)| id == tid);
                let other_unlaunched_exists = fighters.iter().any(|f| {
                    !self.missile_launch_history.iter().any(|&(id, _)| id == f.id)
                });
                if target_in_history && other_unlaunched_exists {
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

        // 4.5. Missile Launching and Radio Broadcasting (Guns 1 & 2)
        let mut fired_missile_this_tick = false;
        let mut fired_target_id = None;

        // Ensure we never fire two missiles within three ticks of each other.
        // This spaces out locks on the single radio channel 3.
        if current_tick - self.last_missile_fired_tick >= 3 {
            let mut launched_slot = None;
            if self.weapon_targets[1].is_some() && reload_ticks(1) == 0 {
                launched_slot = Some(1);
            } else if self.weapon_targets[2].is_some() && reload_ticks(2) == 0 {
                launched_slot = Some(2);
            }

            if let Some(w) = launched_slot {
                if let Some(tid) = self.weapon_targets[w] {
                    fire(w);
                    fired_missile_this_tick = true;
                    fired_target_id = Some(tid);
                    self.last_missile_fired_tick = current_tick;

                    // Update launch history
                    self.missile_launch_history.retain(|&(mid, _)| mid != tid);
                    self.missile_launch_history.push((tid, current_tick));

                    // Clear assignment immediately to target a different enemy on subsequent launch
                    self.weapon_targets[w] = None;
                }
            }
        }

        // Update broadcast state
        if fired_missile_this_tick {
            self.broadcast_target_id = fired_target_id;
            self.broadcast_ticks = 1;
        } else if self.broadcast_target_id.is_some() {
            self.broadcast_ticks += 1;
            if self.broadcast_ticks > 2 {
                self.broadcast_target_id = None;
                self.broadcast_ticks = 0;
            }
        }

        // Broadcast active target telemetry on SecureRadio
        if let Some(tid) = self.broadcast_target_id {
            if let Some(f) = fighters.iter().find(|fighter| fighter.id == tid) {
                let telemetry = TargetTelemetry {
                    position: f.current_position(),
                    velocity: f.current_velocity(),
                };
                self.missile_radio.transmit(0, telemetry.serialize());
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
