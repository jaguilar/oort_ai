use oort_api::prelude::*;
use crate::control::{quick_turn_with_target_omega, AngleTracker};
use crate::missile::MissileGuidance;
use crate::radar::RadarController;

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
        let rc = RadarController::new();
        let mut mg = MissileGuidance::new();
        mg.target_channel = 3;

        // Pre-tune the cruiser's radio slot 0 to target channel 3 to avoid the 1-tick delay
        select_radio(0);
        set_radio_channel(mg.target_channel);

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

        // Broadcast active target telemetry on channel 3
        if let Some(tid) = self.broadcast_target_id {
            if let Some(f) = fighters.iter().find(|fighter| fighter.id == tid) {
                select_radio(0);
                set_radio_channel(3);
                send([f.current_position().x, f.current_position().y, f.current_velocity().x, f.current_velocity().y]);
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
