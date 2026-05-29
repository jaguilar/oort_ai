use oort_api::prelude::*;
use crate::control::{quick_turn_with_target_omega, AngleTracker, MissileGuidance};
use crate::radar::RadarController;

struct ExpectedIntercept {
    position: Vec2,
    expiry_tick: u32,
}

pub struct Ship {
    radar_controller: RadarController,
    weapon_targets: [Option<u32>; 4], // Targets assigned to weapon slots 0, 1, 2, and 3 (missile)
    angle_tracker: AngleTracker,      // Tracks Gun 0 target angle rate for ship orientation
    intercept_plots: Vec<ExpectedIntercept>, // Intercept points to draw for active bullets
    missile_guidance: MissileGuidance,
    just_fired_missile: bool,
    previous_missile_target_id: Option<u32>,
    current_missile_target_id: Option<u32>,
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

        // Pre-tune the frigate's radio slot 0 to target channel 3 to avoid the 1-tick delay
        select_radio(0);
        set_radio_channel(mg.target_channel);

        Ship {
            radar_controller: rc,
            weapon_targets: [None; 4],
            angle_tracker: AngleTracker::new(5.0),
            intercept_plots: Vec::new(),
            missile_guidance: mg,
            just_fired_missile: false,
            previous_missile_target_id: None,
            current_missile_target_id: None,
            missile_launch_history: Vec::new(),
            seen_target_ids: Vec::new(),
        }
    }

    fn is_better_target(&self, w: usize, a: &crate::radar::Contact, b: &crate::radar::Contact) -> bool {
        let num_guns_a = (0..4)
            .filter(|&other_w| other_w != w && self.weapon_targets[other_w] == Some(a.id))
            .count();
        let num_guns_b = (0..4)
            .filter(|&other_w| other_w != w && self.weapon_targets[other_w] == Some(b.id))
            .count();

        let in_missile_a = self.missile_launch_history.iter().any(|&(id, _)| id == a.id);
        let in_missile_b = self.missile_launch_history.iter().any(|&(id, _)| id == b.id);

        let is_clean_a = num_guns_a == 0 && !in_missile_a;
        let is_clean_b = num_guns_b == 0 && !in_missile_b;

        if is_clean_a != is_clean_b {
            return is_clean_a;
        }

        if num_guns_a != num_guns_b {
            return num_guns_a < num_guns_b;
        }

        let ship_pos = position();
        match w {
            0 => {
                let angle_a = (a.position - ship_pos).angle();
                let angle_b = (b.position - ship_pos).angle();
                let diff_a = angle_diff(heading(), angle_a).abs();
                let diff_b = angle_diff(heading(), angle_b).abs();
                diff_a < diff_b
            }
            1 | 2 => {
                let dist_a = ship_pos.distance(a.position);
                let dist_b = ship_pos.distance(b.position);
                dist_a < dist_b
            }
            3 => {
                let t_a = if let Some((t, _)) = predict_lead_exact(
                    ship_pos,
                    velocity(),
                    1000.0,
                    a.position,
                    a.velocity,
                    a.acceleration,
                ) {
                    t
                } else {
                    ship_pos.distance(a.position) / 1000.0 + 1000.0
                };

                let t_b = if let Some((t, _)) = predict_lead_exact(
                    ship_pos,
                    velocity(),
                    1000.0,
                    b.position,
                    b.velocity,
                    b.acceleration,
                ) {
                    t
                } else {
                    ship_pos.distance(b.position) / 1000.0 + 1000.0
                };

                t_a > t_b
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
        for w in 0..4 {
            if let Some(tid) = self.weapon_targets[w] {
                if reload_ticks(w) <= 5 {
                    priority_ids.push(tid);
                }
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
        for w in 0..4 {
            if let Some(tid) = self.weapon_targets[w] {
                if !fighters.iter().any(|f| f.id == tid) {
                    self.weapon_targets[w] = None;
                }
            }
        }

        // Missile exception: if missile's current target was recently launched at,
        // and other unlaunched targets exist, clear it to pick a new one.
        if let Some(tid) = self.weapon_targets[3] {
            let target_in_history = self.missile_launch_history.iter().any(|&(id, _)| id == tid);
            let other_unlaunched_exists = fighters.iter().any(|f| {
                !self.missile_launch_history.iter().any(|&(id, _)| id == f.id)
            });
            if target_in_history && other_unlaunched_exists {
                self.weapon_targets[3] = None;
            }
        }

        // Build list of weapons that are ready and need assignment (currently have None target)
        let mut assignment_order = Vec::new();
        if self.weapon_targets[3].is_none() && reload_ticks(3) <= 4 {
            assignment_order.push(3);
        }
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
                // If we've seen less than 4 targets, do not allow duplicate target assignments.
                if self.seen_target_ids.len() < 4 {
                    let num_guns = (0..4)
                        .filter(|&other_w| other_w != w && self.weapon_targets[other_w] == Some(f.id))
                        .count();
                    if num_guns > 0 {
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
        // Gun 0: Forward-pointing high-velocity gun (Bullet Speed: 4000.0 m/s, Local Offset: [40.0, 0.0])
        if let Some(tid) = self.weapon_targets[0] {
            if let Some(f) = fighters.iter().find(|fighter| fighter.id == tid) {
                const GUN0_BULLET_SPEED: f64 = 4000.0;
                let gun0_offset = Vec2::new(40.0, 0.0);
                let gun0_pos = position() + gun0_offset.rotate(heading());

                if let Some((time_to_impact, lead_dir)) = predict_lead_exact(
                    gun0_pos,
                    velocity(),
                    GUN0_BULLET_SPEED,
                    f.position,
                    f.velocity,
                    f.acceleration,
                ) {
                    let lead_angle = lead_dir.angle();
                    let target_omega = self.angle_tracker.update(lead_angle);
                    quick_turn_with_target_omega(lead_angle, target_omega);

                    // Visualization
                    let p_e = f.position + time_to_impact * f.velocity + 0.5 * f.acceleration * time_to_impact * (time_to_impact + TICK_LENGTH);
                    draw_line(gun0_pos, p_e, rgb(255, 0, 0));
                    draw_square(p_e, 25.0, rgb(255, 0, 0));

                    // Fire when aligned within 1.0 degree of lead angle
                    let diff = angle_diff(heading(), lead_angle);
                    if reload_ticks(0) == 0 && diff.abs() < 0.1f64.to_radians() {
                        fire(0);
                        let expiry_tick = current_tick + (time_to_impact / TICK_LENGTH).round() as u32;
                        self.intercept_plots.push(ExpectedIntercept {
                            position: p_e,
                            expiry_tick,
                        });
                    }
                } else {
                    let direct_angle = (f.position - gun0_pos).angle();
                    let target_omega = self.angle_tracker.update(direct_angle);
                    quick_turn_with_target_omega(direct_angle, target_omega);
                }
            }
        } else {
            // If Gun 0 is unassigned, turn towards one of the other guns' targets if available
            let fallback_target = self.weapon_targets[1]
                .or(self.weapon_targets[2])
                .and_then(|tid| fighters.iter().find(|f| f.id == tid));
            if let Some(f) = fallback_target {
                let direct_angle = (f.position - position()).angle();
                let target_omega = self.angle_tracker.update(direct_angle);
                quick_turn_with_target_omega(direct_angle, target_omega);
            }
        }

        // Guns 1 and 2: Turreted guns (Bullet Speed: 1000.0 m/s, Local Offsets: [0.0, 30.0] and [0.0, -30.0])
        const TURRET_BULLET_SPEED: f64 = 1000.0;
        for &w in &[1, 2] {
            if let Some(tid) = self.weapon_targets[w] {
                if let Some(f) = fighters.iter().find(|fighter| fighter.id == tid) {
                    let gun_offset = if w == 1 {
                        Vec2::new(0.0, 30.0)
                    } else {
                        Vec2::new(0.0, -30.0)
                    };
                    let gun_pos = position() + gun_offset.rotate(heading());

                    if let Some((time_to_impact, lead_dir)) = predict_lead_exact(
                        gun_pos,
                        velocity(),
                        TURRET_BULLET_SPEED,
                        f.position,
                        f.velocity,
                        f.acceleration,
                    ) {
                        let lead_angle = lead_dir.angle();
                        let p_e = f.position + time_to_impact * f.velocity + 0.5 * f.acceleration * time_to_impact * (time_to_impact + TICK_LENGTH);
                        let dist = gun_pos.distance(p_e);
                        let max_angle_offset = if dist > 0.0 { 10.0 / dist } else { 0.0 };
                        let angle_offset = rand(-max_angle_offset, max_angle_offset);
                        let target_angle = lead_angle + angle_offset;
                        aim(w, target_angle);

                        // Visualization
                        draw_line(gun_pos, p_e, rgb(0, 255, 0));
                        draw_triangle(p_e, 20.0, rgb(0, 255, 0));

                        if reload_ticks(w) == 0 {
                            fire(w);
                            let expiry_tick = current_tick + (time_to_impact / TICK_LENGTH).round() as u32;
                            let shot_dir = Vec2::new(target_angle.cos(), target_angle.sin());
                            let randomized_p_e = gun_pos + dist * shot_dir;
                            self.intercept_plots.push(ExpectedIntercept {
                                position: randomized_p_e,
                                expiry_tick,
                            });
                        }
                    } else {
                        let direct_angle = (f.position - gun_pos).angle();
                        aim(w, direct_angle);
                    }
                }
            }
        }

        // 4.5. Radio Broadcasting and Missile Weapon Slot 3
        let mut should_broadcast = false;
        if self.just_fired_missile {
            should_broadcast = true;
            self.just_fired_missile = false;
        }

        if should_broadcast {
            if let Some(tid) = self.current_missile_target_id {
                if Some(tid) != self.previous_missile_target_id {
                    if let Some(f) = fighters.iter().find(|fighter| fighter.id == tid) {
                        select_radio(0);
                        debug!("Frigate broadcasting target telemetry: id={}, pos=({:.1}, {:.1}) vel=({:.1}, {:.1}) on channel {}", f.id, f.position.x, f.position.y, f.velocity.x, f.velocity.y, self.missile_guidance.target_channel);
                        send([f.position.x, f.position.y, f.velocity.x, f.velocity.y]);
                    }
                }
            }
        }

        let mut fired_missile_this_tick = false;
        if let Some(tid) = self.weapon_targets[3] {
            if reload_ticks(3) == 0 {
                fire(3);
                fired_missile_this_tick = true;
                self.missile_launch_history.retain(|&(mid, _)| mid != tid);
                self.missile_launch_history.push((tid, current_tick));
            }
        }

        if fired_missile_this_tick {
            self.previous_missile_target_id = self.current_missile_target_id;
            self.current_missile_target_id = self.weapon_targets[3];
        }
        self.just_fired_missile = fired_missile_this_tick;

        // 5. Draw expected intercept plots for debug
        // Retain only plots whose expected time is in the future
        self.intercept_plots.retain(|plot| current_tick <= plot.expiry_tick);
        for plot in &self.intercept_plots {
            // Draw a small red circle (modeled as an 8-sided polygon with radius 8.0)
            draw_polygon(plot.position, 8.0, 8, 0.0, rgb(255, 0, 0));
        }

        // 6. Draw a blue triangle at each contact's estimated position
        for contact in contacts {
            draw_triangle(contact.position, 15.0, rgb(0, 0, 255));
        }
    }
}
