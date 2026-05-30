use oort_api::prelude::*;
use crate::radar::{RadarController, Contact};

/// Calculates the clamped torque required to turn toward the target angle, taking target angular velocity into account.
pub fn quick_turn_torque_with_target_omega(target_angle: f64, target_omega: f64) -> f64 {
    quick_turn_torque_with_target_omega_impl(
        heading(),
        angular_velocity(),
        max_angular_acceleration(),
        target_angle,
        target_omega,
    )
}

/// A pure implementation of quick_turn_torque_with_target_omega that does not call global functions.
pub fn quick_turn_torque_with_target_omega_impl(
    heading: f64,
    omega: f64,
    max_ang_accel: f64,
    target_angle: f64,
    target_omega: f64,
) -> f64 {
    let difference = angle_diff(heading, target_angle - target_omega * TICK_LENGTH);
    let unaccelerated_next_heading = heading + omega * TICK_LENGTH;
    let target_angle_next = target_angle;
    let diff_next = angle_diff(unaccelerated_next_heading, target_angle_next);
    let omega_rel = omega - target_omega;
    
    // 1-step deadbeat control when error is already zero (or extremely small)
    if difference.abs() <= 1e-9 && omega_rel.abs() <= max_ang_accel * TICK_LENGTH {
        return (-omega_rel / TICK_LENGTH).clamp(-max_ang_accel, max_ang_accel);
    }

    // 2-step deadbeat control
    if difference.abs() <= max_ang_accel * TICK_LENGTH * TICK_LENGTH
        && diff_next.abs() <= max_ang_accel * TICK_LENGTH * TICK_LENGTH
    {
        let alpha_req = diff_next / (TICK_LENGTH * TICK_LENGTH);
        return alpha_req.clamp(-max_ang_accel, max_ang_accel);
    }
    
    // No safety buffer: use 100% of max angular acceleration
    let a_dec = max_ang_accel;
    let k_p = 60.0;
    
    let theta_trans = a_dec / (k_p * k_p);
    let theta_offset = theta_trans / 2.0;
    
    let omega_target_static = if difference.abs() <= theta_trans {
        k_p * difference
    } else {
        difference.signum() * (2.0 * a_dec * (difference.abs() - theta_offset)).sqrt()
    };
    
    let is_decelerating = difference * (omega - target_omega) > 0.0
        && (omega - target_omega).abs() > omega_target_static.abs()
        && difference.abs() > theta_trans;

    let alpha_req = if is_decelerating {
        let s = difference.signum();
        let diff_adjusted = difference.abs() - theta_offset;
        ( (target_omega - omega) - a_dec * s * TICK_LENGTH - s * (a_dec * (2.0 * diff_adjusted + a_dec * TICK_LENGTH * TICK_LENGTH)).sqrt() ) / TICK_LENGTH
    } else {
        let omega_target = omega_target_static + target_omega;
        (omega_target - omega) / TICK_LENGTH
    };
    
    let torque = alpha_req.clamp(-max_ang_accel, max_ang_accel);
    if difference.abs() > 0.002 {
        if torque >= 0.0 { max_ang_accel } else { -max_ang_accel }
    } else {
        torque
    }
}



/// Calculates the clamped torque required to turn toward the target angle without overshooting.
pub fn quick_turn_torque(target_angle: f64) -> f64 {
    quick_turn_torque_with_target_omega(target_angle, 0.0)
}

/// Turn at the maximum possible speed for a given ship that will not overshoot the target angle, taking target angular velocity into account.
pub fn quick_turn_with_target_omega(target_angle: f64, target_omega: f64) {
    let omega = angular_velocity();
    let max_ang_accel = max_angular_acceleration();
    let unaccelerated_next_heading = heading() + omega * TICK_LENGTH;
    let diff_next = angle_diff(unaccelerated_next_heading, target_angle);
    let speed_diff = (omega - target_omega).abs();
    
    if diff_next.abs() <= max_ang_accel * TICK_LENGTH * TICK_LENGTH
        && speed_diff <= max_ang_accel * TICK_LENGTH
    {
        debug!(
            "Exact match with target heading expected next tick. Planned heading: {}",
            format_sig_figs(target_angle, 6)
        );
    }

    torque(quick_turn_torque_with_target_omega(target_angle, target_omega));
}

/// Turn at the maximum possible speed for a given ship that will not overshoot the target angle.
pub fn quick_turn(target_angle: f64) {
    quick_turn_with_target_omega(target_angle, 0.0);
}


/// A general Newton's method root-finding solver.
/// Finds a value $x$ such that $f(x) \approx 0$.
pub fn newton_solve<F, DF, C>(
    mut x: f64,
    f: F,
    df: DF,
    clamp: C,
    max_iter: usize,
    min_precision: f64,
) -> Option<f64>
where
    F: Fn(f64) -> f64,
    DF: Fn(f64) -> f64,
    C: Fn(f64) -> f64,
{
    for _ in 0..max_iter {
        x = clamp(x);
        let fx = f(x);
        if fx.abs() < min_precision {
            return Some(x);
        }
        let dfx = df(x);
        if dfx.abs() < 1e-12 {
            break;
        }
        let step = fx / dfx;
        x -= step;
        if step.abs() < min_precision {
            return Some(x);
        }
    }
    Some(clamp(x))
}

/// Predicts the lead direction and time-to-impact of a bullet fired
/// from a ship at a target under constant acceleration, taking into account
/// the discrete nature of Oort physics and gun offset.
/// 
/// Returns `Option<(f64, Vec2)>` representing the time-to-impact and the required direction.
pub fn predict_lead(
    our_pos: Vec2,
    our_vel: Vec2,
    bullet_speed: f64,
    target_pos: Vec2,
    target_vel: Vec2,
    target_accel: Vec2,
) -> Option<(f64, Vec2)> {
    let dp0 = target_pos - our_pos;
    let r_len = dp0.length();
    if r_len < 1e-6 {
        return None;
    }
    let dv = target_vel - our_vel;
    let v_c = -dv.dot(dp0) / r_len;
    let t0 = r_len / (bullet_speed + v_c.max(0.0));
    
    let f = |t: f64| {
        let p_e = target_pos + t * target_vel + 0.5 * target_accel * t * (t + TICK_LENGTH);
        let d = p_e - our_pos - t * our_vel;
        d.length() - bullet_speed * t
    };

    let df = |t: f64| {
        let p_e = target_pos + t * target_vel + 0.5 * target_accel * t * (t + TICK_LENGTH);
        let d = p_e - our_pos - t * our_vel;
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

    if let Some(t) = newton_solve(t0, f, df, clamp, 20, 1e-4) {
        if t >= 0.0 {
            let p_e = target_pos + t * target_vel + 0.5 * target_accel * t * (t + TICK_LENGTH);
            let d = p_e - our_pos - t * our_vel;
            if d.length() > 0.0 {
                return Some((t, d.normalize()));
            }
        }
    }
    None
}

/// Helper to track the angular velocity of the target direction using EWMA.
pub struct AngleTracker {
    prev_angle: Option<f64>,
    omega_ewma: f64,
    alpha: f64,
}

impl AngleTracker {
    pub fn new(time_constant_ticks: f64) -> Self {
        Self {
            prev_angle: None,
            omega_ewma: 0.0,
            alpha: 1.0 / time_constant_ticks,
        }
    }

    pub fn update(&mut self, current_angle: f64) -> f64 {
        let current_omega = match self.prev_angle {
            Some(prev_angle) => angle_diff(prev_angle, current_angle) / TICK_LENGTH,
            None => 0.0,
        };
        self.prev_angle = Some(current_angle);
        self.omega_ewma = self.alpha * current_omega + (1.0 - self.alpha) * self.omega_ewma;
        self.omega_ewma
    }

    pub fn omega(&self) -> f64 {
        self.omega_ewma
    }
}

/// Tracks the estimated position, velocity, and acceleration of a target over ticks.
pub struct TargetTracker {
    last_seen_tick: Option<u32>,
    position: Vec2,
    velocity: Vec2,
    acceleration: Vec2,
}

impl TargetTracker {
    pub fn new() -> Self {
        Self {
            last_seen_tick: None,
            position: Vec2::new(0.0, 0.0),
            velocity: Vec2::new(0.0, 0.0),
            acceleration: Vec2::new(0.0, 0.0),
        }
    }

    pub fn update(&mut self, current_tick: u32, pos: Vec2, vel: Vec2) {
        if let Some(last_tick) = self.last_seen_tick {
            let dt = (current_tick - last_tick) as f64 * TICK_LENGTH;
            if dt > 0.0 {
                self.acceleration = (vel - self.velocity) / dt;
            }
        } else {
            self.acceleration = Vec2::new(0.0, 0.0);
        }
        self.position = pos;
        self.velocity = vel;
        self.last_seen_tick = Some(current_tick);
    }

    pub fn position(&self) -> Vec2 {
        self.position
    }

    pub fn velocity(&self) -> Vec2 {
        self.velocity
    }

    pub fn acceleration(&self) -> Vec2 {
        self.acceleration
    }

    pub fn last_seen_tick(&self) -> Option<u32> {
        self.last_seen_tick
    }

    /// Extrapolates the target's position and velocity at the current tick if we didn't scan it this tick.
    pub fn extrapolate(&self, current_tick: u32) -> (Vec2, Vec2) {
        if let Some(last_tick) = self.last_seen_tick {
            let dt = (current_tick - last_tick) as f64 * TICK_LENGTH;
            let pred_vel = self.velocity + self.acceleration * dt;
            let pred_pos = self.position + self.velocity * dt + 0.5 * self.acceleration * dt * dt;
            (pred_pos, pred_vel)
        } else {
            (self.position, self.velocity)
        }
    }
}

/// Missile guidance system encapsulating radar scanning, target selection,
/// proportional navigation, fuel economy, and terminal orientation control.
pub struct MissileGuidance {
    // Adjustable constant parameters
    pub proximity_dist: f64,
    pub proximity_ticks: f64,
    pub pn_gain: f64,
    pub pn_min_vc: f64,
    pub target_lock_delay_ticks: u32,
    pub fuel_economy_dv_threshold: f64,
    pub min_search_fuel: f64,
    pub turn_safety_buffer_ticks: f64,

    // State
    pub radar_controller: RadarController,
    pub angle_tracker: AngleTracker,
    pub initial_fuel: f64,
    pub target_id: Option<u32>,
    pub first_detection_tick: Option<u32>,
    pub target_channel: usize,
    pub radio_target_tracker: TargetTracker,
}

impl MissileGuidance {
    pub fn new() -> Self {
        // Initial setup for the missile's radar and radio
        select_radio(0);
        set_radio_channel(3);

        if class() == Class::Missile {
            select_radar(0);
            set_radar_heading(heading());
        }

        Self {
            proximity_dist: 20.0,
            proximity_ticks: 5.0,
            pn_gain: 4.0,
            pn_min_vc: 100.0,
            target_lock_delay_ticks: 22,
            fuel_economy_dv_threshold: 500.0,
            min_search_fuel: 500.0,
            turn_safety_buffer_ticks: 1.0,

            radar_controller: RadarController::new(),
            angle_tracker: AngleTracker::new(5.0),
            initial_fuel: fuel(),
            target_id: None,
            first_detection_tick: None,
            target_channel: 3,
            radio_target_tracker: TargetTracker::new(),
        }
    }

    pub fn tick(&mut self) {
        // 1. Listen on the target radio channel
        select_radio(0);
        set_radio_channel(self.target_channel);

        let mut radio_ping = None;

        // Try standard float message first ([f64; 4]: pos_x, pos_y, vel_x, vel_y)
        if let Some(msg) = receive() {
            let pos_x = msg[0];
            let pos_y = msg[1];
            let vel_x = msg[2];
            let vel_y = msg[3];

            // Mismatched byte representation when interpreted as f64 yields astronomical values or NaNs.
            // Check that the numbers are finite and lie within reasonable limits to distinguish formats.
            if pos_x.is_finite() && pos_y.is_finite() && vel_x.is_finite() && vel_y.is_finite()
                && pos_x.abs() < 100_000.0 && pos_y.abs() < 100_000.0
                && vel_x.abs() < 10_000.0 && vel_y.abs() < 10_000.0
            {
                let pos = vec2(pos_x, pos_y);
                let vel = vec2(vel_x, vel_y);
                self.radio_target_tracker.update(current_tick(), pos, vel);
                let accel = self.radio_target_tracker.acceleration();
                debug!("Decoded radio float ping on channel {}: pos=({:.1}, {:.1}) vel=({:.1}, {:.1})", self.target_channel, pos.x, pos.y, vel.x, vel.y);
                radio_ping = Some((pos, vel, accel));
            }
        }

        // Set current target as high priority in the radar controller
        self.radar_controller.priority_targets = self.target_id.map(|id| vec![id]).unwrap_or_default();

        // Update radar controller
        self.radar_controller.update();

        // 2. Insert/update target in radar contact database if received via radio
        let mut radio_target_id = None;
        if let Some((pos, vel, accel)) = radio_ping {
            let mut has_good_lock = false;
            if let Some(tid) = self.target_id {
                if let Some(contact) = self.radar_controller.contacts().iter().find(|c| c.id == tid) {
                    if 3.89 * contact.current_pos_uncertainty() <= 50.0 {
                        has_good_lock = true;
                    }
                }
            }

            if !has_good_lock {
                let contact_id = self.radar_controller.update_from_radio(pos, vel, accel, self.target_id);
                radio_target_id = Some(contact_id);
                if let Some(old_id) = self.target_id {
                    if old_id != contact_id {
                        debug!("Ceasing targeting of target {} to lock onto radio target {} (reason: current target has poor lock or is lost)", old_id, contact_id);
                    }
                }
                self.target_id = Some(contact_id);
            } else {
                debug!("Ignoring radio telemetry: current target has a good lock (CI within 50m).");
            }
        }

        let contacts = self.radar_controller.contacts();

        // 3. Target selection
        if radio_target_id.is_none() {
            // A target is valid if it is still tracked in the contact list and is a Fighter
            let target_still_valid = if let Some(id) = self.target_id {
                contacts.iter().any(|c| c.id == id && c.class == Class::Fighter)
            } else {
                false
            };

            // Filter contacts to only target Class::Fighter
            let fighters: Vec<&Contact> = contacts.iter()
                .filter(|c| c.class == Class::Fighter)
                .collect();

            // Set the first detection tick if we've just detected a fighter
            if !fighters.is_empty() && self.first_detection_tick.is_none() {
                self.first_detection_tick = Some(current_tick());
            }

            if !target_still_valid {
                // Delay target selection until we've had time for two full scans after first target detection
                let can_lock = if let Some(first_tick) = self.first_detection_tick {
                    current_tick() - first_tick >= self.target_lock_delay_ticks
                } else {
                    false
                };

                if can_lock && !fighters.is_empty() {
                    // Pick a random fighter instead of the closest one
                    let idx = (rand(0.0, fighters.len() as f64).floor() as usize).min(fighters.len() - 1);
                    let new_id = fighters[idx].id;
                    if let Some(old_id) = self.target_id {
                        debug!("Ceasing targeting of target {} because it is no longer valid (not in contacts list or not a fighter); locking onto new target {}", old_id, new_id);
                    }
                    self.target_id = Some(new_id);
                } else {
                    if let Some(old_id) = self.target_id {
                        debug!("Ceasing targeting of target {} because it is no longer valid (not in contacts list or not a fighter) and no new target lock could be acquired", old_id);
                    }
                    self.target_id = None;
                }
            }
        }

        if let Some(tid) = self.target_id {
            if let Some(target) = contacts.iter().find(|c| c.id == tid) {
                let target_pos = target.current_position();
                let target_vel = target.current_velocity();
                let target_class = target.class;

                let r = target_pos - position();
                let r_len = r.length();
                let v_rel = target_vel - velocity();

                // 1. Self-destruct proximity check: detonate if within target proximity or will be soon
                let next_r = r + v_rel * (self.proximity_ticks * TICK_LENGTH);
                if r_len < self.proximity_dist || next_r.length() < self.proximity_dist {
                    explode();
                    return;
                }

                // 2. Proportional Navigation Guidance
                // Line-of-sight angular rate (cross product / r^2)
                let numerator = r.x * v_rel.y - r.y * v_rel.x;
                let denominator = r.dot(r);
                let los_rate = if denominator > 1e-6 { numerator / denominator } else { 0.0 };

                // Closing velocity
                let v_c = -v_rel.dot(r) / r_len;

                // Lateral acceleration command perpendicular to LOS in the direction of rotation
                let e_perp = vec2(-r.y, r.x) / r_len;
                let a_lateral = self.pn_gain * v_c.max(self.pn_min_vc) * los_rate * e_perp;

                // Forward acceleration with fuel economy check
                let dir = if r_len > 1e-6 {
                    r / r_len
                } else {
                    vec2(heading().cos(), heading().sin())
                };

                let expended_fuel = self.initial_fuel - fuel();
                let possible_enemy_dv = if v_c > 0.0 {
                    let t_intercept = r_len / v_c;
                    let base_dv = t_intercept * target_class.default_stats().max_forward_acceleration;
                    let boost_dv = (t_intercept / 10.0).ceil() * 100.0;
                    base_dv + boost_dv
                } else {
                    0.0
                };

                // Engages if we have expended at least threshold delta v and remaining fuel is low
                let fuel_economy = expended_fuel >= self.fuel_economy_dv_threshold && fuel() < possible_enemy_dv;

                let forward_acc = if fuel_economy {
                    0.0
                } else {
                    max_forward_acceleration()
                };

                let a_total = a_lateral + dir * forward_acc;


                // Turn to point directly at target intercept point when it's time to explode
                let time_to_intercept = if v_c > 0.0 { r_len / v_c } else { f64::MAX };
                let time_until_explosion = (time_to_intercept - self.proximity_ticks * TICK_LENGTH).max(0.0);

                // Heading we need to face at the moment of explosion
                let position_at_explosion = position() + time_until_explosion * velocity();
                let target_pos_at_explosion = target.position_at(current_tick() + (time_until_explosion / TICK_LENGTH).round() as u32);
                let explode_heading = (target_pos_at_explosion - position_at_explosion).angle();

                // Calculate how long we need to turn from the current heading to explode_heading
                let diff = angle_diff(heading(), explode_heading);
                let omega = angular_velocity();
                let a = max_angular_acceleration().max(1.0);

                let time_to_stop = if omega * diff < 0.0 { omega.abs() / a } else { 0.0 };
                let angle_to_stop = 0.5 * omega.powi(2) / a;
                let remaining_angle = (diff.abs() + if omega * diff < 0.0 { angle_to_stop } else { -angle_to_stop }).max(0.0);
                let time_remaining_turn = 2.0 * (remaining_angle / a).sqrt();
                let turn_time = time_to_stop + time_remaining_turn;

                // Add a small safety buffer to ensure we finish the turn in time
                let safety_buffer = self.turn_safety_buffer_ticks * TICK_LENGTH;
                let turn_time_with_buffer = turn_time + safety_buffer;

                let torque_cmd = if time_until_explosion <= turn_time_with_buffer {
                    optimal_turn_torque(position(), velocity(), target_pos_at_explosion, None)
                } else {
                    let target_pos_aim = position() + a_total;
                    optimal_turn_torque(position(), velocity(), target_pos_aim, Some(velocity()))
                };
                torque(torque_cmd);

                accelerate(a_total);

                // Print guidance mode and acceleration components
                let is_terminal = time_until_explosion <= turn_time_with_buffer;
                let mode = if is_terminal {
                    "Terminal Turn"
                } else if fuel_economy {
                    "Fuel Economy"
                } else {
                    "Standard PN Guidance"
                };
                debug!("Mode: {}", mode);
                debug!("Acc X: {:.2}", a_total.x);
                debug!("Acc Y: {:.2}", a_total.y);
                debug!("Lat Acc X: {:.2}, Y: {:.2}", a_lateral.x, a_lateral.y);
                debug!("Fwd Acc X: {:.2}, Y: {:.2}", (dir * forward_acc).x, (dir * forward_acc).y);

                // Boost to reach target faster, but only if not in fuel economy mode
                if !fuel_economy {
                    activate_ability(Ability::Boost);
                }

                // Draw projected intercept point of the currently selected target
                if v_c > 0.0 {
                    let intercept_point = target.position_at(current_tick() + (time_to_intercept / TICK_LENGTH).round() as u32);
                    draw_diamond(intercept_point, 16.0, rgb(255, 0, 0));
                    draw_line(position(), intercept_point, rgb(255, 0, 0));
                    draw_text!(intercept_point + vec2(0.0, 20.0), rgb(255, 0, 0), "Intercept: {:.2}s", time_to_intercept);
                }

                // Draw the current position of the currently selected target
                draw_square(target_pos, 20.0, rgb(255, 0, 0));
            }
        } else {
            // No target - burn straight ahead at maximum speed until we find a lock, provided we retain fuel
            let (mode, a_cmd) = if fuel() >= self.min_search_fuel {
                let heading_dir = vec2(heading().cos(), heading().sin());
                ("Search Mode", heading_dir * max_forward_acceleration())
            } else {
                ("Coast Mode", vec2(0.0, 0.0))
            };
            debug!("Mode: {}", mode);
            debug!("Acc X: {:.2}", a_cmd.x);
            debug!("Acc Y: {:.2}", a_cmd.y);

            accelerate(a_cmd);
            if mode == "Search Mode" {
                activate_ability(Ability::Boost);
            } else {
                deactivate_ability(Ability::Boost);
            }
        }
    }
}

/// Computes the optimal constant-acceleration vector to intercept a target,
/// given our position and velocity, and the target's position, velocity, and acceleration.
pub fn optimal_intercept_acceleration(
    our_pos: Vec2,
    our_vel: Vec2,
    target_pos: Vec2,
    target_vel: Vec2,
    target_accel: Vec2,
    max_accel: f64,
) -> Option<Vec2> {
    let x = target_pos - our_pos;
    let v_rel = target_vel - our_vel;

    // We want to solve for time-to-impact T > 0 in:
    // |2x/T^2 + 2v_rel/T + target_accel|^2 = max_accel^2
    // Which is f(T) = |P(T)|^2 - max_accel^2 * T^4 = 0
    // where P(T) = 2x + 2v_rel*T + target_accel*T^2
    let dist = x.length();
    let rel_speed = v_rel.length();
    let mut t = if rel_speed > 1.0 {
        dist / rel_speed
    } else {
        (2.0 * dist / max_accel).sqrt()
    };
    t = t.max(0.01);

    let f = |t: f64| {
        let p = 2.0 * x + 2.0 * v_rel * t + target_accel * t * t;
        p.dot(p) - max_accel.powi(2) * t.powi(4)
    };

    let df = |t: f64| {
        let p = 2.0 * x + 2.0 * v_rel * t + target_accel * t * t;
        let p_prime = 2.0 * v_rel + 2.0 * target_accel * t;
        2.0 * p.dot(p_prime) - 4.0 * max_accel.powi(2) * t.powi(3)
    };

    let clamp = |t: f64| t.max(0.001);

    if let Some(time_to_impact) = newton_solve(t, f, df, clamp, 30, 1e-4) {
        if time_to_impact > 0.0 {
            let u = (2.0 * x) / (time_to_impact * time_to_impact) + (2.0 * v_rel) / time_to_impact + target_accel;
            if u.length() > 0.0 {
                return Some(u.normalize() * max_accel);
            }
        }
    }
    None
}

/// Calculates the optimal direct angular acceleration (torque) required to turn to face a target point
/// in the minimum amount of time, taking into account target and self velocities to compute the
/// exact relative angular velocity (line-of-sight rate) with zero lag.
pub fn optimal_turn_torque(
    our_pos: Vec2,
    our_vel: Vec2,
    target_pos: Vec2,
    target_vel: Option<Vec2>,
) -> f64 {
    let r = target_pos - our_pos;
    let v_rel = target_vel.unwrap_or(Vec2::new(0.0, 0.0)) - our_vel;
    let target_angle = r.angle();
    let r_len_sq = r.dot(r);
    let target_omega = if r_len_sq > 1e-6 {
        (r.x * v_rel.y - r.y * v_rel.x) / r_len_sq
    } else {
        0.0
    };

    let difference = angle_diff(heading(), target_angle);
    let omega = angular_velocity();
    let max_ang_accel = max_angular_acceleration();

    let unaccelerated_next_heading = heading() + omega * TICK_LENGTH;
    let diff_next = angle_diff(unaccelerated_next_heading, target_angle);
    let speed_diff = (omega - target_omega).abs();

    if diff_next.abs() <= max_ang_accel * TICK_LENGTH * TICK_LENGTH
        && speed_diff <= max_ang_accel * TICK_LENGTH
    {
        let alpha_req = diff_next / (TICK_LENGTH * TICK_LENGTH);
        return alpha_req.clamp(-max_ang_accel, max_ang_accel);
    }

    let a_dec = max_ang_accel * 0.98;
    let k_p = 10.0;
    let theta_trans = a_dec / (k_p * k_p);
    let theta_offset = theta_trans / 2.0;

    let omega_target_static = if difference.abs() <= theta_trans {
        k_p * difference
    } else {
        difference.signum() * (2.0 * a_dec * (difference.abs() - theta_offset)).sqrt()
    };

    let is_decelerating = difference * (omega - target_omega) > 0.0
        && (omega - target_omega).abs() > omega_target_static.abs()
        && difference.abs() > theta_trans;

    let alpha_req = if is_decelerating {
        let s = difference.signum();
        let diff_adjusted = difference.abs() - theta_offset;
        ( (target_omega - omega) - a_dec * s * TICK_LENGTH - s * (a_dec * (2.0 * diff_adjusted + a_dec * TICK_LENGTH * TICK_LENGTH)).sqrt() ) / TICK_LENGTH
    } else {
        let omega_target = omega_target_static + target_omega;
        (omega_target - omega) / TICK_LENGTH
    };
    
    alpha_req.clamp(-max_ang_accel, max_ang_accel)
}

/// Estimates the time to complete a turn to face a target angle and match target angular velocity,
/// assuming we use the quick_turn_torque_with_target_omega controller.
pub fn quick_turn_time_with_target_omega(target_angle: f64, target_omega: f64) -> f64 {
    let mut x0 = angle_diff(heading(), target_angle - target_omega * TICK_LENGTH);
    let mut v0 = angular_velocity() - target_omega;
    let a = max_angular_acceleration();
    if a < 1e-6 {
        return f64::INFINITY;
    }

    let a_dec = a * 0.98;
    let k_p = 10.0;
    let theta_trans = a_dec / (k_p * k_p);
    let theta_offset = theta_trans / 2.0;

    // Check if we are already aligned and matched within the 1-tick control window
    let unaccelerated_next_heading = heading() + angular_velocity() * TICK_LENGTH;
    let diff_next = angle_diff(unaccelerated_next_heading, target_angle);
    let speed_diff = (angular_velocity() - target_omega).abs();
    if diff_next.abs() <= a * TICK_LENGTH * TICK_LENGTH
        && speed_diff <= a * TICK_LENGTH
    {
        return 0.0;
    }

    if x0 < 0.0 {
        x0 = -x0;
        v0 = -v0;
    }

    // Scale the settling time dynamically based on the error size, up to a maximum of 3.0 / k_p
    let x_start = x0 + v0.abs() / k_p;
    let tol = 0.001; // tolerance in radians
    let t_settle = if x_start > tol {
        ((x_start / tol).ln() / k_p).min(3.0 / k_p)
    } else {
        0.0
    };

    if x0 <= theta_trans {
        let v_target = -k_p * x0;
        let t1 = (v0 - v_target).abs() / a;
        t1 + t_settle
    } else {
        let d = (a_dec / (a + a_dec)) * v0 * v0 + (2.0 * a * a_dec / (a + a_dec)) * (x0 - theta_offset);
        let d_sqrt = d.max(0.0).sqrt();
        let t1 = ((v0 + d_sqrt) / a).max(0.0);
        let t2 = ((d_sqrt - k_p * theta_trans) / a_dec).max(0.0);
        t1 + t2 + t_settle
    }
}

/// Helper function to format a floating point number to a specific number of significant figures in standard decimal notation.
pub fn format_sig_figs(val: f64, n: usize) -> String {
    if val == 0.0 || !val.is_finite() {
        return format!("{:.1$}", val, n - 1);
    }
    let abs_val = val.abs();
    let log10_val = abs_val.log10();
    let mut d = log10_val.floor() as isize + 1;
    let mut precision = (n as isize - d).max(0) as usize;
    
    let factor = 10.0f64.powi(precision as i32);
    let rounded = (abs_val * factor).round() / factor;
    if rounded != 0.0 {
        let rounded_d = rounded.log10().floor() as isize + 1;
        if rounded_d != d {
            d = rounded_d;
            precision = (n as isize - d).max(0) as usize;
        }
    }
    format!("{:.1$}", val, precision)
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Lcg {
        state: u64,
    }

    impl Lcg {
        fn new(seed: u64) -> Self {
            Self { state: seed }
        }

        fn next_u64(&mut self) -> u64 {
            self.state = self.state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            self.state
        }

        fn next_f64(&mut self) -> f64 {
            (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
        }

        fn gen_range(&mut self, min: f64, max: f64) -> f64 {
            min + self.next_f64() * (max - min)
        }

        fn gen_vec2(&mut self, min_mag: f64, max_mag: f64) -> (f64, f64) {
            let mag = self.gen_range(min_mag, max_mag);
            let angle = self.gen_range(-std::f64::consts::PI, std::f64::consts::PI);
            (mag * angle.cos(), mag * angle.sin())
        }
    }

    fn wrap_angle(angle: f64) -> f64 {
        let mut a = angle % (2.0 * std::f64::consts::PI);
        if a > std::f64::consts::PI {
            a -= 2.0 * std::f64::consts::PI;
        } else if a < -std::f64::consts::PI {
            a += 2.0 * std::f64::consts::PI;
        }
        a
    }

    fn angle_diff_local(from: f64, to: f64) -> f64 {
        let diff = (to - from) % (2.0 * std::f64::consts::PI);
        let diff = if diff > std::f64::consts::PI {
            diff - 2.0 * std::f64::consts::PI
        } else if diff < -std::f64::consts::PI {
            diff + 2.0 * std::f64::consts::PI
        } else {
            diff
        };
        diff
    }

    #[test]
    fn test_quick_turn_torque_simulation() {
        let mut lcg = Lcg::new(1337);
        let max_torque = 2.0 * std::f64::consts::PI;
        let max_accel = 60.0;
        let dt = 1.0 / 60.0;
        let epsilon = 1e-4;
        let max_ticks = 240;

        let mut successes = 0;
        let mut failures = 0;
        let mut max_torque_failures = 0;
        let mut overshoot_failures = 0;
        let mut convergence_failures = 0;

        #[derive(Debug)]
        struct FailureInfo {
            case_idx: usize,
            reason: &'static str,
            turn_ticks: usize,
            max_torque_ticks_before_align: usize,
            min_required: usize,
            has_overshot: bool,
            unavoidable: bool,
            max_overshoot: f64,
            init_err: f64,
            init_omega_rel: f64,
        }
        let mut failure_details = Vec::new();

        for case_idx in 0..1000 {
            let mut p_pos = Vec2::new(0.0, 0.0);

            let dist = lcg.gen_range(1000.0, 20000.0);
            let target_start_angle = lcg.gen_range(-std::f64::consts::PI, std::f64::consts::PI);
            let mut t_pos = Vec2::new(dist * target_start_angle.cos(), dist * target_start_angle.sin());

            let (p_vx, p_vy) = lcg.gen_vec2(0.0, 100.0);
            let mut p_vel = Vec2::new(p_vx, p_vy);

            let (t_vx, t_vy) = lcg.gen_vec2(0.0, 100.0);
            let mut t_vel = Vec2::new(t_vx, t_vy);

            let (t_ax, t_ay) = lcg.gen_vec2(0.0, max_accel);
            let t_accel = Vec2::new(t_ax, t_ay);

            let mut p_omega = lcg.gen_range(-max_torque, max_torque);
            let mut p_heading = lcg.gen_range(-std::f64::consts::PI, std::f64::consts::PI);

            let p_accel_mode = case_idx % 3;
            let (p_ax_const, p_ay_const) = if p_accel_mode == 1 {
                lcg.gen_vec2(0.0, max_accel)
            } else {
                (0.0, 0.0)
            };
            let p_accel_const = Vec2::new(p_ax_const, p_ay_const);

            let mut converged = false;
            let mut consecutive_aligned = 0;
            let mut turn_ticks = 0;

            // Track initial conditions for overshoot checking
            let initial_r = t_pos - p_pos;
            let initial_v_rel = t_vel - p_vel;
            let initial_target_heading = initial_r.angle();
            let initial_target_omega = if initial_r.dot(initial_r) > 1e-6 {
                (initial_r.x * initial_v_rel.y - initial_r.y * initial_v_rel.x) / initial_r.dot(initial_r)
            } else {
                0.0
            };
            let initial_error = angle_diff_local(p_heading, initial_target_heading);
            let initial_omega_rel = p_omega - initial_target_omega;

            let mut torques = Vec::new();
            let mut has_overshot = false;
            let mut max_overshoot: f64 = 0.0;
            let mut overshoot_tick = None;
            let mut prev_error: Option<f64> = None;

            for tick in 0..max_ticks {
                let r = t_pos - p_pos;
                let v_rel = t_vel - p_vel;
                let target_heading = r.angle();
                let r_len_sq = r.dot(r);
                let target_omega = if r_len_sq > 1e-6 {
                    (r.x * v_rel.y - r.y * v_rel.x) / r_len_sq
                } else {
                    0.0
                };

                let heading_error = angle_diff_local(p_heading, target_heading);

                if heading_error.abs() <= epsilon {
                    consecutive_aligned += 1;
                } else {
                    consecutive_aligned = 0;
                }

                if consecutive_aligned >= 3 {
                    if !converged {
                        converged = true;
                        turn_ticks = tick - 2; // First tick of alignment
                    }
                }

                // Overshoot detection: sign change of heading error
                if let Some(prev_err) = prev_error {
                    if prev_err * heading_error < 0.0 && heading_error.abs() < 0.2 {
                        has_overshot = true;
                        if overshoot_tick.is_none() {
                            overshoot_tick = Some(tick);
                        }
                    }
                }
                prev_error = Some(heading_error);

                if has_overshot {
                    max_overshoot = max_overshoot.max(heading_error.abs());
                }

                let target_heading_next = target_heading + target_omega * dt;
                let torque_cmd = quick_turn_torque_with_target_omega_impl(
                    p_heading,
                    p_omega,
                    max_torque,
                    target_heading_next,
                    target_omega,
                );

                torques.push(torque_cmd);

                // Update target physics
                t_vel = t_vel + t_accel * dt;
                t_pos = t_pos + t_vel * dt;

                // Update player physics
                let p_accel = match p_accel_mode {
                    0 => Vec2::new(0.0, 0.0),
                    1 => p_accel_const,
                    _ => Vec2::new(max_accel * p_heading.cos(), max_accel * p_heading.sin()),
                };
                p_vel = p_vel + p_accel * dt;
                p_pos = p_pos + p_vel * dt;

                let torque = torque_cmd.clamp(-max_torque, max_torque);
                p_omega = p_omega + torque * dt;
                p_heading = wrap_angle(p_heading + p_omega * dt);
            }

            if converged {
                let mut is_valid = true;
                let mut reason = "";

                // Check if overshoot was unavoidable (max deceleration commanded on all ticks up to the overshoot)
                let unavoidable_overshoot = if has_overshot {
                    if let Some(ot) = overshoot_tick {
                        torques.iter().take(ot + 1).all(|&tq| {
                            if initial_omega_rel > 0.0 {
                                tq <= -max_torque + 1e-9
                            } else if initial_omega_rel < 0.0 {
                                tq >= max_torque - 1e-9
                            } else {
                                false
                            }
                        })
                    } else {
                        false
                    }
                } else {
                    false
                };

                if has_overshot && !unavoidable_overshoot && max_overshoot > 0.35 {
                    overshoot_failures += 1;
                    is_valid = false;
                    reason = "Overshoot";
                }

                // Calculate minimum required max torque ticks during the active turning phase
                let min_required_max_torque_ticks = (0.9 * turn_ticks as f64).max((turn_ticks as isize - 6) as f64).ceil() as usize;

                let mut max_torque_ticks_before_align = 0;
                for &tq in torques.iter().take(turn_ticks) {
                    if tq.abs() >= max_torque - 1e-9 {
                        max_torque_ticks_before_align += 1;
                    }
                }

                if max_torque_ticks_before_align < min_required_max_torque_ticks {
                    max_torque_failures += 1;
                    is_valid = false;
                    reason = if reason.is_empty() { "Max Torque Ticks" } else { "Overshoot & Max Torque" };
                }

                if is_valid {
                    successes += 1;
                } else {
                    failures += 1;
                    failure_details.push(FailureInfo {
                        case_idx,
                        reason,
                        turn_ticks,
                        max_torque_ticks_before_align,
                        min_required: min_required_max_torque_ticks,
                        has_overshot,
                        unavoidable: unavoidable_overshoot,
                        max_overshoot,
                        init_err: initial_error,
                        init_omega_rel: initial_omega_rel,
                    });
                }
            } else {
                failures += 1;
                convergence_failures += 1;
                failure_details.push(FailureInfo {
                    case_idx,
                    reason: "Did Not Converge",
                    turn_ticks: max_ticks,
                    max_torque_ticks_before_align: 0,
                    min_required: 0,
                    has_overshot,
                    unavoidable: false,
                    max_overshoot,
                    init_err: initial_error,
                    init_omega_rel: initial_omega_rel,
                });
            }
        }

        println!("Successful test cases: {} / 1000", successes);
        println!("Failures: {}, Max Torque Failures: {}, Overshoot Failures: {}, Convergence Failures: {}",
            failures, max_torque_failures, overshoot_failures, convergence_failures);
        
        for (i, detail) in failure_details.iter().take(50).enumerate() {
            println!("Failure #{}: {:?}", i, detail);
        }

        if failures > 0 {
            panic!("Test failed: {} test cases failed the requirements.", failures);
        }
    }

    #[test]
    fn test_quick_turn_torque_perpendicular_traversing() {
        let mut lcg = Lcg::new(42);
        let max_torque = 2.0 * std::f64::consts::PI;
        let dt = 1.0 / 60.0;
        let epsilon = 1e-4;
        let max_ticks = 600; // 10 seconds simulation

        for case_idx in 0..20 {
            // Ship starts at origin, stationary, with randomized initial heading
            let p_pos = Vec2::new(0.0, 0.0);
            let init_heading = lcg.gen_range(-std::f64::consts::PI, std::f64::consts::PI);
            let mut p_heading = init_heading;
            let mut p_omega = 0.0;

            // Enemy starts 15 km away along X axis (heading 0.0)
            // Traversing perpendicular (along Y axis) at randomized speed between 100 and 300 m/s
            let speed = lcg.gen_range(100.0, 300.0);
            let direction = if lcg.next_f64() > 0.5 { 1.0 } else { -1.0 };
            let mut t_pos = Vec2::new(15000.0, 0.0);
            let t_vel = Vec2::new(0.0, direction * speed);

            let mut converged = false;
            let mut consecutive_aligned = 0;
            let mut turn_ticks = None;

            for tick in 0..max_ticks {
                let r = t_pos - p_pos;
                let target_heading = r.angle();
                let r_len_sq = r.dot(r);
                let target_omega = if r_len_sq > 1e-6 {
                    (r.x * t_vel.y - r.y * t_vel.x) / r_len_sq
                } else {
                    0.0
                };

                let heading_error = angle_diff_local(p_heading, target_heading);
                let omega_error = p_omega - target_omega;

                // We consider the tracking successful if the error is extremely small
                if heading_error.abs() <= epsilon && omega_error.abs() <= 1e-4 {
                    consecutive_aligned += 1;
                } else {
                    consecutive_aligned = 0;
                }

                if consecutive_aligned >= 10 && turn_ticks.is_none() {
                    converged = true;
                    turn_ticks = Some(tick - 9);
                }

                // Target next heading estimation for the torque controller
                let target_heading_next = target_heading + target_omega * dt;
                let torque_cmd = quick_turn_torque_with_target_omega_impl(
                    p_heading,
                    p_omega,
                    max_torque,
                    target_heading_next,
                    target_omega,
                );

                // Update physics
                // Enemy moves at a fixed velocity
                t_pos = t_pos + t_vel * dt;

                // Ship only rotates (p_pos is stationary)
                let torque = torque_cmd.clamp(-max_torque, max_torque);
                p_omega = p_omega + torque * dt;
                p_heading = wrap_angle(p_heading + p_omega * dt);
            }

            assert!(
                converged,
                "Case {}: Should converge to target angle and track it with zero lag. Init heading: {:.4} rad, speed: {:.1} m/s",
                case_idx, init_heading, speed
            );
            let tt = turn_ticks.unwrap();
            println!(
                "Case {}: Converged in {} ticks ({:.2}s). Init heading: {:.4} rad, speed: {:.1} m/s",
                case_idx, tt, tt as f64 * dt, init_heading, speed
            );
        }
    }
}
