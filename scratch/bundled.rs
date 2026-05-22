pub mod control { // start multifile
use oort_api::prelude::*;

/// Calculates the clamped torque required to turn toward the target angle without overshooting.
pub fn quick_turn_torque(target_angle: f64) -> f64 {
    let difference = angle_diff(heading(), target_angle);
    let omega = angular_velocity();
    let max_ang_accel = max_angular_acceleration();
    
    // Safety buffer: use 98% of max angular acceleration to prevent any overshoot
    let a_dec = max_ang_accel * 0.98;
    let k_p = 10.0;
    
    let theta_trans = a_dec / (k_p * k_p);
    let theta_offset = theta_trans / 2.0;
    
    let omega_target = if difference.abs() <= theta_trans {
        k_p * difference
    } else {
        difference.signum() * (2.0 * a_dec * (difference.abs() - theta_offset)).sqrt()
    };
    
    let alpha_req = (omega_target - omega) / TICK_LENGTH;
    alpha_req.clamp(-max_ang_accel, max_ang_accel)
}

/// Turn at the maximum possible speed for a given ship that will not overshoot the target angle.
pub fn quick_turn(target_angle: f64) {
    torque(quick_turn_torque(target_angle));
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
    let t0 = dp0.length() / bullet_speed;
    
    let f = |t: f64| {
        let p_e = target_pos + t * target_vel + 0.5 * target_accel * t * (t + TICK_LENGTH);
        let d = p_e - our_pos - t * our_vel;
        d.length() - (bullet_speed * t + 20.0)
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


} // end multifile
pub mod TUTORIAL_GUNS { // start multifile
use oort_api::prelude::*;
use crate::control::quick_turn;

pub struct Ship {
    initial_direction: Option<Vec2>,
}

impl Ship {
    pub fn new() -> Ship {
        Ship {
            initial_direction: None,
        }
    }

    pub fn tick(&mut self) {
        let initial_direction = *self.initial_direction.get_or_insert_with(|| {
            let h = heading();
            Vec2::new(h.cos(), h.sin())
        });

        // Determine target direction and angle.
        let target_pos = target();
        let to_target = target_pos - position();
        
        let (_target_direction, _target_angle) = if to_target.length() > 0.0 {
            const BULLET_SPEED: f64 = 1000.0;
            let a = to_target.dot(to_target);
            let b = to_target.dot(velocity());
            let c = velocity().dot(velocity()) - BULLET_SPEED * BULLET_SPEED;
            let discriminant = b * b - a * c;
            
            if discriminant >= 0.0 {
                let k = (b + discriminant.sqrt()) / a;
                let target_vec = to_target * k - velocity();
                if target_vec.length() > 0.0 {
                    (target_vec.normalize(), target_vec.angle())
                } else {
                    (to_target.normalize(), to_target.angle())
                }
            } else {
                (to_target.normalize(), to_target.angle())
            }
        } else {
            let h = heading();
            (initial_direction, h)
        };

        // Determine target direction.
        let target_dir = if to_target.length() > 0.0 {
            to_target.normalize()
        } else {
            initial_direction
        };

        // Compute desired acceleration vector to align velocity with target direction
        let v_perp = velocity() - target_dir * velocity().dot(target_dir);
        
        // Use a moderate feedback gain to correct drift over a longer timescale, preventing oscillation
        let lateral_gain = 2.0;
        let a_perp_req = -v_perp * lateral_gain;
        let a_perp_len = a_perp_req.length();

        let a_limit = max_forward_acceleration() + 100.0;
        let desired_accel = if a_perp_len >= a_limit {
            a_perp_req.normalize() * a_limit
        } else {
            let a_para_len = (a_limit * a_limit - a_perp_len * a_perp_len).sqrt();
            a_perp_req + target_dir * a_para_len
        };

        let target_angle_base = target_dir.angle();
        let accel_angle_raw = desired_accel.angle();
        let angle_diff_accel = angle_diff(target_angle_base, accel_angle_raw);
        
        // Clamp the over-turn angle to a maximum of 30 degrees to prevent overshooting under boost
        let max_overturn = 30.0f64.to_radians();
        let clamped_diff = angle_diff_accel.clamp(-max_overturn, max_overturn);
        let accel_angle = target_angle_base + clamped_diff;

        // Turn towards the desired acceleration direction to maximize thrust along it (over-turning if off-target)
        quick_turn(accel_angle);

        // Calculate required acceleration vectors considering limits and boost
        let b = 100.0; // Boost acceleration magnitude (100 m/s²)
        let a_f_max = max_forward_acceleration();
        let a_b_max = max_backward_acceleration();
        let a_l_max = max_lateral_acceleration();

        let delta_theta = angle_diff(heading(), accel_angle);
        let cos_dt = delta_theta.cos();
        let sin_dt = delta_theta.sin();

        // Calculate net acceleration capability when boost is active
        let mut k_min_boost = 0.0f64;
        let mut k_max_boost = f64::INFINITY;

        if cos_dt > 0.0 {
            k_min_boost = k_min_boost.max((b - a_b_max) / cos_dt);
            k_max_boost = k_max_boost.min((b + a_f_max) / cos_dt);
        } else if cos_dt < 0.0 {
            k_min_boost = k_min_boost.max((b + a_f_max) / cos_dt);
            k_max_boost = k_max_boost.min((b - a_b_max) / cos_dt);
        }

        if sin_dt.abs() > 0.0 {
            k_max_boost = k_max_boost.min(a_l_max / sin_dt.abs());
        }

        let (boost_f, boost_l) = if k_min_boost <= k_max_boost && k_max_boost > 0.0 {
            let k_opt = k_max_boost;
            (k_opt * cos_dt - b, k_opt * sin_dt)
        } else {
            let a_l_val = if sin_dt > 0.0 { a_l_max } else { -a_l_max };
            (-a_b_max, a_l_val)
        };

        let net_f = boost_f + b;
        let net_l = boost_l;
        let net_angle_local = net_l.atan2(net_f);
        let angle_diff_net_target = angle_diff(net_angle_local, delta_theta);
        let can_point_within_1_deg = angle_diff_net_target.abs() < 60.0f64.to_radians();

        // 1. Activate Boost if the net vector can point within 1 degree of target
        if can_point_within_1_deg {
            activate_ability(Ability::Boost);
        }

        let is_boost_active = active_abilities().get_ability(Ability::Boost);

        let (a_f, a_l) = if is_boost_active {
            // 2. When boost is active, use the calculated vector to accelerate towards target
            (boost_f, boost_l)
        } else {
            // 3. When boost is inactive, accelerate directly toward the target
            let mut k_max = f64::INFINITY;
            if cos_dt > 0.0 {
                k_max = k_max.min(a_f_max / cos_dt);
            } else if cos_dt < 0.0 {
                k_max = k_max.min(-a_b_max / cos_dt);
            }

            if sin_dt.abs() > 0.0 {
                k_max = k_max.min(a_l_max / sin_dt.abs());
            }

            let k_opt = if k_max.is_finite() && k_max > 0.0 { k_max } else { a_f_max };
            (k_opt * cos_dt, k_opt * sin_dt)
        };

        // Convert ship-local acceleration components (a_f, a_l) back to world coordinates
        let heading_dir = Vec2::new(heading().cos(), heading().sin());
        let left_dir = Vec2::new(-heading().sin(), heading().cos());
        accelerate(heading_dir * a_f + left_dir * a_l);

        // Selective firing: only shoot if bullet vector is within 0.1 degrees of target
        const BULLET_SPEED: f64 = 1000.0;
        let heading_vec = Vec2::new(heading().cos(), heading().sin());
        let bullet_absolute_vel = velocity() + heading_vec * BULLET_SPEED;
        let bullet_angle_diff = angle_diff(bullet_absolute_vel.angle(), to_target.angle());
        
        if reload_ticks(0) == 0 && bullet_angle_diff.abs() < 1.0f64.to_radians() {
            fire(0);
        }
    }
}

} // end multifile
pub mod TUTORIAL_ROTATION { // start multifile
use oort_api::prelude::*;
use crate::control::quick_turn;

pub struct Ship;

impl Ship {
    pub fn new() -> Ship {
        Ship
    }

    pub fn tick(&mut self) {
        // Zero acceleration since we don't want to accelerate toward the target.
        accelerate(vec2(0.0, 0.0));

        let target_pos = target();
        let to_target = target_pos - position();
        
        let target_angle = if to_target.length() > 0.0 {
            // General lead prediction formula (incorporating target and ship velocities)
            const BULLET_SPEED: f64 = 1000.0;
            let rel_vel = target_velocity() - velocity();
            
            let a = to_target.dot(to_target);
            let b = to_target.dot(rel_vel);
            let c = rel_vel.dot(rel_vel) - BULLET_SPEED * BULLET_SPEED;
            let discriminant = b * b - a * c;
            
            if discriminant >= 0.0 {
                let k = (-b + discriminant.sqrt()) / a;
                let target_vec = to_target * k + rel_vel;
                if target_vec.length() > 0.0 {
                    target_vec.angle()
                } else {
                    to_target.angle()
                }
            } else {
                to_target.angle()
            }
        } else {
            heading()
        };

        // Turn to target as quickly as possible
        quick_turn(target_angle);

        // Fire when aimed within 0.5 degrees
        let difference = angle_diff(heading(), target_angle);
        if reload_ticks(0) == 0 && difference.abs() < 0.5f64.to_radians() {
            fire(0);
        }
    }
}

} // end multifile
pub mod TUTORIAL_LEAD { // start multifile
use oort_api::prelude::*;
use crate::control::{quick_turn, predict_lead};

pub struct Ship {
    prev_target_vel: Option<Vec2>,
}

impl Ship {
    pub fn new() -> Ship {
        Ship {
            prev_target_vel: None,
        }
    }

    pub fn tick(&mut self) {
        // Zero linear acceleration for this scenario.
        let our_accel = Vec2::new(0.0, 0.0);
        accelerate(our_accel);

        // Estimate target acceleration
        let current_target_vel = target_velocity();
        let target_accel = match self.prev_target_vel {
            Some(prev_vel) => (current_target_vel - prev_vel) / TICK_LENGTH,
            None => Vec2::new(0.0, 0.0),
        };
        self.prev_target_vel = Some(current_target_vel);

        const BULLET_SPEED: f64 = 1000.0;

        debug!("--- TICK ---");
        debug!("Our pos: {:?}, vel: {:?}, heading: {:.2} deg", position(), velocity(), heading().to_degrees());
        debug!("Target pos: {:?}, vel: {:?}, accel: {:?}", target(), target_velocity(), target_accel);

        // Predict optimal lead target angle and time-to-impact using current states
        if let Some((time_to_impact, lead_dir)) = predict_lead(
            position(),
            velocity(),
            BULLET_SPEED,
            target(),
            target_velocity(),
            target_accel,
        ) {
            let target_angle = lead_dir.angle();
            let diff = angle_diff(heading(), target_angle);

            debug!("Lead sol: time={:.4}s, dir={:?}, target_angle={:.2} deg (diff={:.2} deg)", time_to_impact, lead_dir, target_angle.to_degrees(), diff.to_degrees());

            // Direct rotation command to face the target angle
            quick_turn(target_angle);

            // Fire if the weapon is ready and our current heading is aligned within 0.5 degrees
            debug!("Reload ticks: {}", reload_ticks(0));
            if reload_ticks(0) == 0 && diff.abs() < 0.5f64.to_radians() {
                debug!("FIRED!");
                fire(0);
            }
        } else {
            debug!("No lead solution found!");
            // Fallback if no solution found: turn toward target position
            let to_target = target() - position();
            quick_turn(to_target.angle());
        }
    }
}

} // end multifile

pub use TUTORIAL_LEAD::*;
