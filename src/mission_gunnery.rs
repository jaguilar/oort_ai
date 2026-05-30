use std::collections::HashMap;
use oort_api::prelude::*;
use crate::control::{quick_turn_with_target_omega, AngleTracker, quick_turn_time_with_target_omega, predict_lead};
use crate::radar::{RadarController, DefaultScanSliceGenerator, ScanSliceGenerator, ScanSlice, Contact};

struct ExpectedIntercept {
    position: Vec2,
    expiry_tick: u32,
}

pub struct Ship {
    radar_controller: RadarController,
    weapon_target: Option<u32>,        // Target assigned to Gun 0
    angle_tracker: AngleTracker,       // Tracks Gun 0 target angle rate for ship orientation
    intercept_plots: Vec<ExpectedIntercept>, // Intercept points to draw for active bullets
    fire_counts: HashMap<u32, u32>,    // Tracks count of shots fired on each target ID
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

pub struct GunneryScanSliceGenerator {
    max_distance: f64,
    current_slice_index: usize,
    last_slice_tick: Option<u32>,
    min_distance: f64,
}

impl GunneryScanSliceGenerator {
    pub fn new(max_distance: f64) -> Self {
        Self {
            max_distance,
            current_slice_index: 0,
            last_slice_tick: None,
            min_distance: 0.0,
        }
    }
}

impl ScanSliceGenerator for GunneryScanSliceGenerator {
    fn next_slice(&mut self, _target: Option<&Contact>) -> ScanSlice {
        let current_t = current_tick();
        let mut hit = false;
        let mut hit_distance = 0.0;

        if let Some(last_tick) = self.last_slice_tick {
            if last_tick + 1 == current_t {
                select_radar(0);
                if let Some(r) = scan() {
                    hit = true;
                    hit_distance = position().distance(r.position);
                }
            }
        }

        if hit {
            self.min_distance = hit_distance + 10.0;
            debug!(
                "Scan hit at {:.1}m in slice {}. Repeating scan with min_distance = {:.1}m",
                hit_distance, self.current_slice_index, self.min_distance
            );
        } else {
            if self.last_slice_tick.is_some() && self.last_slice_tick.unwrap() + 1 == current_t {
                self.current_slice_index = (self.current_slice_index + 1) % 10;
            }
            self.min_distance = 0.0;
        }

        let slice_width = TAU / 80.0;
        let center_angle = -TAU / 8.0 + (self.current_slice_index as f64 + 0.5) * slice_width;

        let slice = ScanSlice {
            angle: center_angle,
            width: slice_width,
            min_distance: self.min_distance,
            max_distance: self.max_distance,
        };

        self.last_slice_tick = Some(current_t);
        slice
    }
}

impl Ship {
    pub fn new() -> Ship {
        let mut rc = RadarController::new();
        rc.slice_generator = Box::new(GunneryScanSliceGenerator::new(30000.0));
        rc.priority_track_interval = 3;

        Ship {
            radar_controller: rc,
            weapon_target: None,
            angle_tracker: AngleTracker::new(5.0),
            intercept_plots: Vec::new(),
            fire_counts: HashMap::new(),
        }
    }

    fn target_aim_time(&self, c: &crate::radar::Contact) -> f64 {
        let gun0_offset = Vec2::new(40.0, 0.0);
        let gun0_pos = position() + gun0_offset.rotate(heading());
        const GUN0_BULLET_SPEED: f64 = 4000.0;

        let (target_angle, target_omega) = if let Some((_, lead_dir)) = predict_lead(
            gun0_pos,
            velocity(),
            GUN0_BULLET_SPEED,
            c.current_position(),
            c.current_velocity(),
            c.acceleration,
        ) {
            let target_angle = lead_dir.angle();
            let r = c.current_position() - position();
            let v_rel = c.current_velocity() - velocity();
            let r_len_sq = r.dot(r);
            let target_omega = if r_len_sq > 1e-6 {
                (r.x * v_rel.y - r.y * v_rel.x) / r_len_sq
            } else {
                0.0
            };
            (target_angle, target_omega)
        } else {
            let r = c.current_position() - position();
            let v_rel = c.current_velocity() - velocity();
            let target_angle = r.angle();
            let r_len_sq = r.dot(r);
            let target_omega = if r_len_sq > 1e-6 {
                (r.x * v_rel.y - r.y * v_rel.x) / r_len_sq
            } else {
                0.0
            };
            (target_angle, target_omega)
        };

        quick_turn_time_with_target_omega(target_angle, target_omega)
    }

    fn target_firing_solution_y(&self, c: &crate::radar::Contact) -> f64 {
        let gun0_offset = Vec2::new(40.0, 0.0);
        let gun0_pos = position() + gun0_offset.rotate(heading());
        const GUN0_BULLET_SPEED: f64 = 4000.0;

        if let Some((t, _)) = predict_lead(
            gun0_pos,
            velocity(),
            GUN0_BULLET_SPEED,
            c.current_position(),
            c.current_velocity(),
            c.acceleration,
        ) {
            let p_e = c.position_at(current_tick() + (t / TICK_LENGTH).round() as u32);
            p_e.y
        } else {
            c.current_position().y
        }
    }

    fn is_better_target(&self, a: &crate::radar::Contact, b: &crate::radar::Contact) -> bool {
        let y_a = self.target_firing_solution_y(a);
        let y_b = self.target_firing_solution_y(b);

        // Hysteresis: apply a bonus to the currently tracked target's Y value
        let hysteresis_bonus = 500.0; // in meters
        let y_a_adj = y_a + if Some(a.id) == self.weapon_target { hysteresis_bonus } else { 0.0 };
        let y_b_adj = y_b + if Some(b.id) == self.weapon_target { hysteresis_bonus } else { 0.0 };

        y_a_adj > y_b_adj
    }

    pub fn tick(&mut self) {
        let current_tick = current_tick();

        // 1. Update priority targets list based on weapon targeting (always mark current target as high priority)
        let mut priority_ids = Vec::new();
        if let Some(tid) = self.weapon_target {
            priority_ids.push(tid);
        }
        self.radar_controller.priority_targets = priority_ids;

        // 2. Update radar scheduler and contact database
        self.radar_controller.update();

        // 3. Fetch active contacts from the radar controller
        let contacts = self.radar_controller.contacts();

        // 4. Update Weapon Assignment
        // Select the tracked contact with the minimum number of firing attempts.
        // If attempts are equal, choose the target that is furthest north in its firing solution.
        let mut best_target: Option<&crate::radar::Contact> = None;
        for c in contacts {
            let turn_time = self.target_aim_time(c);
            let sol_y = self.target_firing_solution_y(c);
            let ci_c = 3.89 * c.current_pos_uncertainty();
            debug!("Target {} considered: turn time = {:.3}s, firing sol Y = {:.1}m, CI = {:.1}m att={}", c.id, turn_time, sol_y, ci_c, *self.fire_counts.get(&c.id).unwrap_or(&0));

            if let Some(best) = best_target {
                let bad_c = ci_c > 50.0;
                let bad_best = 3.89 * best.current_pos_uncertainty() > 50.0;

                if bad_best && !bad_c {
                    best_target = Some(c);
                } else if !bad_best && bad_c {
                    // Keep best, do nothing
                } else {
                    let attempts_c = *self.fire_counts.get(&c.id).unwrap_or(&0);
                    let attempts_best = *self.fire_counts.get(&best.id).unwrap_or(&0);
                    if attempts_c < attempts_best {
                        best_target = Some(c);
                    } else if attempts_c == attempts_best {
                        if self.is_better_target(c, best) {
                            best_target = Some(c);
                        }
                    }
                }
            } else {
                best_target = Some(c);
            }
        }
        self.weapon_target = best_target.map(|c| c.id);

        if let Some(tid) = self.weapon_target {
            debug!("Selected target: {}", tid);
        } else {
            debug!("Selected target: None");
        }

        // 5. Weapon Aiming and Firing
        // Gun 0: Forward-pointing high-velocity gun (Bullet Speed: 4000.0 m/s, Local Offset: [40.0, 0.0])
        if let Some(tid) = self.weapon_target {
            if let Some(c) = contacts.iter().find(|contact| contact.id == tid) {
                const GUN0_BULLET_SPEED: f64 = 4000.0;
                let gun0_offset = Vec2::new(40.0, 0.0);
                let gun0_pos = position() + gun0_offset.rotate(heading());

                if let Some((time_to_impact, lead_dir)) = predict_lead_exact(
                    gun0_pos,
                    velocity(),
                    GUN0_BULLET_SPEED,
                    c.current_position(),
                    c.current_velocity(),
                    c.acceleration,
                ) {
                    let lead_angle = lead_dir.angle();
                    let target_omega = self.angle_tracker.update(lead_angle);
                    quick_turn_with_target_omega(lead_angle, target_omega);

                    // Visualization
                    let p_e = c.position_at(current_tick + (time_to_impact / TICK_LENGTH).round() as u32);
                    draw_line(gun0_pos, p_e, rgb(255, 0, 0));
                    draw_square(p_e, 25.0, rgb(255, 0, 0));

                    // Fire when aligned such that the bullet passes within 2 meters of the firing solution
                    // Never fire on anything where the 99.99% confidence interval is more than 20m in size
                    let bullet_pos_at_impact = gun0_pos + time_to_impact * velocity() + time_to_impact * GUN0_BULLET_SPEED * vec2(heading().cos(), heading().sin());
                    let pass_dist = bullet_pos_at_impact.distance(p_e);
                    let ci_size = 3.89 * c.current_pos_uncertainty();
                    
                    let gun_ready = reload_ticks(0) == 0;
                    let aligned = pass_dist <= 1.0;
                    let locked_on = ci_size <= 20.0;

                    if gun_ready && aligned && locked_on {
                        let mut actual_target_id = c.id;
                        let mut collision_tick = (time_to_impact / TICK_LENGTH).round() as u32;
                        let mut collision_pos = p_e;
                        let target_dist = position().distance(c.current_position());
                        let mut min_collision_t = f64::MAX;

                        for other in contacts {
                            if other.id != c.id {
                                let other_dist = position().distance(other.current_position());
                                if other_dist < target_dist {
                                    if let Some((t_other, _)) = predict_lead_exact(
                                        gun0_pos,
                                        velocity(),
                                        GUN0_BULLET_SPEED,
                                        other.current_position(),
                                        other.current_velocity(),
                                        other.acceleration,
                                    ) {
                                        let p_e_other = other.position_at(current_tick + (t_other / TICK_LENGTH).round() as u32);
                                        let p_b_other = gun0_pos + t_other * velocity() + t_other * GUN0_BULLET_SPEED * vec2(heading().cos(), heading().sin());
                                        let pass_dist_other = p_b_other.distance(p_e_other);
                                        let collides = pass_dist_other <= 15.0;
                                        debug!("Bullet test vs closer target {}: intercept dist = {:.2}m, collides = {}", other.id, pass_dist_other, collides);
                                        if collides && t_other < min_collision_t {
                                            min_collision_t = t_other;
                                            actual_target_id = other.id;
                                            collision_tick = (t_other / TICK_LENGTH).round() as u32;
                                            collision_pos = p_e_other;
                                        }
                                    } else {
                                        debug!("Bullet test vs closer target {}: no firing solution found", other.id);
                                    }
                                }
                            }
                        }

                        fire(0);
                        *self.fire_counts.entry(actual_target_id).or_insert(0) += 1;
                        let expiry_tick = current_tick + collision_tick;
                        self.intercept_plots.push(ExpectedIntercept {
                            position: collision_pos,
                            expiry_tick,
                        });
                    } else {
                        let mut reasons = Vec::new();
                        if !gun_ready {
                            reasons.push(format!("reloading ({} ticks left)", reload_ticks(0)));
                        }
                        if !aligned {
                            reasons.push(format!("not aligned (pass_dist = {:.2}m > 1.0m)", pass_dist));
                        }
                        if !locked_on {
                            reasons.push(format!("high target uncertainty (CI = {:.1}m > 20.0m)", ci_size));
                        }
                        debug!("Did not fire on target {}: {}", tid, reasons.join(", "));
                    }
                } else {
                    let direct_angle = (c.current_position() - gun0_pos).angle();
                    let target_omega = self.angle_tracker.update(direct_angle);
                    quick_turn_with_target_omega(direct_angle, target_omega);
                    debug!("Did not fire on target {}: lead prediction failed", tid);
                }
            }
        }

        // 6. Draw expected intercept plots for debug
        self.intercept_plots.retain(|plot| current_tick <= plot.expiry_tick);
        for plot in &self.intercept_plots {
            draw_polygon(plot.position, 8.0, 8, 0.0, rgb(255, 0, 0));
        }

        // 7. Draw a blue triangle at each contact's estimated position
        for contact in contacts {
            draw_triangle(contact.current_position(), 15.0, rgb(0, 0, 255));
        }
    }
}
