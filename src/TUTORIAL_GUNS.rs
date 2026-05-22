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
