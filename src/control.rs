use oort_api::prelude::*;

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
        ((target_omega - omega)
            - a_dec * s * TICK_LENGTH
            - s * (a_dec * (2.0 * diff_adjusted + a_dec * TICK_LENGTH * TICK_LENGTH)).sqrt())
            / TICK_LENGTH
    } else {
        let omega_target = omega_target_static + target_omega;
        (omega_target - omega) / TICK_LENGTH
    };

    let torque = alpha_req.clamp(-max_ang_accel, max_ang_accel);
    if difference.abs() > 0.002 {
        if torque >= 0.0 {
            max_ang_accel
        } else {
            -max_ang_accel
        }
    } else {
        torque
    }
}

/// A pure implementation of the kinematics-based quick turn torque controller using Newton's method.
pub fn quick_turn_torque_kinematic_impl(
    heading: f64,
    omega: f64,
    max_ang_accel: f64,
    our_pos: Vec2,
    our_vel: Vec2,
    our_accel: Vec2,
    target_pos: Vec2,
    target_vel: Vec2,
    target_accel: Vec2,
) -> f64 {
    // 1. Predict target relative state for the deadbeat control checks
    let p_rel_next = target_pos - our_pos
        + (target_vel - our_vel) * TICK_LENGTH
        + 0.5 * (target_accel - our_accel) * TICK_LENGTH * (TICK_LENGTH + TICK_LENGTH);
    let target_heading_next = p_rel_next.angle();
    let r_len_sq_next = p_rel_next.dot(p_rel_next);
    let target_omega_next = if r_len_sq_next > 1e-6 {
        let v_rel_next = target_vel - our_vel + (target_accel - our_accel) * TICK_LENGTH;
        (p_rel_next.x * v_rel_next.y - p_rel_next.y * v_rel_next.x) / r_len_sq_next
    } else {
        0.0
    };

    let difference = angle_diff(
        heading,
        target_heading_next - target_omega_next * TICK_LENGTH,
    );
    let unaccelerated_next_heading = heading + omega * TICK_LENGTH;
    let diff_next = angle_diff(unaccelerated_next_heading, target_heading_next);
    let omega_rel = omega - target_omega_next;

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

    // 2. Newton's root-finding method to find the optimal alignment time T
    let s = if difference.abs() > 1e-9 {
        difference.signum()
    } else if omega_rel.abs() > 1e-9 {
        -omega_rel.signum()
    } else {
        1.0
    };

    let target_heading_0 = (target_pos - our_pos).angle();
    let r_len_sq_0 = (target_pos - our_pos).dot(target_pos - our_pos);
    let init_diff = angle_diff(heading, target_heading_0);
    let target_heading_0_unwrapped = heading + init_diff;

    let target_omega_0 = if r_len_sq_0 > 1e-6 {
        let v_rel_0 = target_vel - our_vel;
        ((target_pos - our_pos).x * v_rel_0.y - (target_pos - our_pos).y * v_rel_0.x) / r_len_sq_0
    } else {
        0.0
    };

    let t0 = quick_turn_time_with_target_omega_pure(
        heading,
        omega,
        max_ang_accel,
        target_heading_0,
        target_omega_0,
    );

    let cross = |a: Vec2, b: Vec2| a.x * b.y - a.y * b.x;

    let f = |t_align: f64| {
        let p_rel = target_pos - our_pos
            + (target_vel - our_vel) * t_align
            + 0.5 * (target_accel - our_accel) * t_align * (t_align + TICK_LENGTH);
        let target_heading = p_rel.angle();
        let target_heading_unwrapped = target_heading
            - 2.0
                * std::f64::consts::PI
                * ((target_heading - target_heading_0_unwrapped) / (2.0 * std::f64::consts::PI))
                    .round();

        let r_len_sq = p_rel.dot(p_rel);
        let target_omega = if r_len_sq > 1e-6 {
            let v_rel = target_vel - our_vel + (target_accel - our_accel) * t_align;
            cross(p_rel, v_rel) / r_len_sq
        } else {
            0.0
        };

        let t = 0.5 * (t_align + (target_omega - omega) / (s * max_ang_accel));
        let theta_our = heading
            + omega * t_align
            + s * max_ang_accel
                * (2.0 * t * t_align - t * t - 0.5 * t_align * t_align
                    + (t - 0.5 * t_align) * TICK_LENGTH);

        target_heading_unwrapped - theta_our
    };

    let df = |t_align: f64| {
        let p_rel = target_pos - our_pos
            + (target_vel - our_vel) * t_align
            + 0.5 * (target_accel - our_accel) * t_align * (t_align + TICK_LENGTH);
        let r_len_sq = p_rel.dot(p_rel);

        let (target_omega, alpha_target) = if r_len_sq > 1e-6 {
            let v_rel = target_vel - our_vel + (target_accel - our_accel) * t_align;
            let a_rel = target_accel - our_accel;
            let omega_t = cross(p_rel, v_rel) / r_len_sq;
            let alpha_t = (cross(p_rel, a_rel) - 2.0 * omega_t * p_rel.dot(v_rel)) / r_len_sq;
            (omega_t, alpha_t)
        } else {
            (0.0, 0.0)
        };

        let t = 0.5 * (t_align + (target_omega - omega) / (s * max_ang_accel));
        let t_dec = t_align - t;

        -(s * max_ang_accel + alpha_target) * t_dec - 0.5 * alpha_target * TICK_LENGTH
    };

    let clamp = |t_align: f64| t_align.max(0.001);

    // Solve for alignment time t_align using newton's method with step clamping
    let mut x = t0;
    let mut solved_t_align = t0;
    for _iter in 0..30 {
        x = clamp(x);
        let fx = f(x);
        if fx.abs() < 1e-4 {
            solved_t_align = x;
            break;
        }
        let dfx = df(x);
        if dfx.abs() < 1e-12 {
            solved_t_align = x;
            break;
        }
        let step = (fx / dfx).clamp(-0.5, 0.5);
        x -= step;
        if step.abs() < 1e-4 {
            solved_t_align = x;
            break;
        }
        solved_t_align = x;
    }
    let t_align = clamp(solved_t_align);

    // Compute the duration of the acceleration phase
    let p_rel = target_pos - our_pos
        + (target_vel - our_vel) * t_align
        + 0.5 * (target_accel - our_accel) * t_align * (t_align + TICK_LENGTH);
    let r_len_sq = p_rel.dot(p_rel);
    let target_omega = if r_len_sq > 1e-6 {
        let v_rel = target_vel - our_vel + (target_accel - our_accel) * t_align;
        cross(p_rel, v_rel) / r_len_sq
    } else {
        0.0
    };

    let t = 0.5 * (t_align + (target_omega - omega) / (s * max_ang_accel));

    let torque = if t > 0.0 {
        s * max_ang_accel
    } else {
        -s * max_ang_accel
    };

    // To prevent rapid chattering/limit-cycles near the target angle and maintain smooth deadbeat handoff,
    // we clamp the torque to deadbeat limits or smooth output when we are very close to alignment.
    if difference.abs() > 0.002 {
        torque
    } else {
        let k_p = 60.0;
        let omega_target = k_p * difference + target_omega_0;
        let torque_fallback = (omega_target - omega) / TICK_LENGTH;
        torque_fallback.clamp(-max_ang_accel, max_ang_accel)
    }
}

/// Calculates the clamped torque required to turn toward the target position using kinematics.
pub fn quick_turn_torque_kinematic(
    target_pos: Vec2,
    target_vel: Vec2,
    target_accel: Vec2,
    our_accel: Vec2,
) -> f64 {
    // Transform target next-tick kinematics to target current-tick (t = 0) kinematics for the solver.
    let target_pos_start = target_pos - target_vel * TICK_LENGTH;
    let target_vel_start = target_vel - target_accel * TICK_LENGTH;
    quick_turn_torque_kinematic_impl(
        heading(),
        angular_velocity(),
        max_angular_acceleration(),
        position(),
        velocity(),
        our_accel,
        target_pos_start,
        target_vel_start,
        target_accel,
    )
}

/// Turn at the maximum possible speed for a given ship that will not overshoot the target angle, using relative kinematics.
pub fn quick_turn_kinematic(
    target_pos: Vec2,
    target_vel: Vec2,
    target_accel: Vec2,
    our_accel: Vec2,
) {
    torque(quick_turn_torque_kinematic(
        target_pos,
        target_vel,
        target_accel,
        our_accel,
    ));
}

/// Calculates the clamped torque required to turn toward the target angle without overshooting.
pub fn quick_turn_torque(target_angle: f64) -> f64 {
    quick_turn_torque_with_target_omega(target_angle, 0.0)
}

/// Turn at the maximum possible speed for a given ship that will not overshoot the target angle, taking target angular velocity into account.
pub fn quick_turn_with_target_omega(target_angle: f64, target_omega: f64) {
    let omega = angular_velocity();
    let _max_ang_accel = max_angular_acceleration();
    let unaccelerated_next_heading = heading() + omega * TICK_LENGTH;
    let _diff_next = angle_diff(unaccelerated_next_heading, target_angle);
    let _speed_diff = (omega - target_omega).abs();

    let p = position();
    draw_line(
        p,
        p + vec2(target_angle.cos(), target_angle.sin()) * 1000.0,
        rgb(0, 255, 0),
    );
    draw_line(
        p,
        p + vec2(heading().cos(), heading().sin()) * 1000.0,
        rgb(0, 0, 255),
    );
    let cmd_torque = quick_turn_torque_with_target_omega(target_angle, target_omega);
    debug!(
        "qt: h={} t={} w={} tw={} => {}",
        heading().to_degrees() as i32,
        target_angle.to_degrees() as i32,
        angular_velocity().to_degrees() as i32,
        target_omega.to_degrees() as i32,
        cmd_torque.to_degrees() as i32
    );

    torque(cmd_torque);
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
    for _i in 0..max_iter {
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
        if t < 0.0 { 0.0 } else { t }
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

/// Estimates the time to complete a turn to face a target angle and match target angular velocity,
/// assuming we use the quick_turn_torque_with_target_omega controller.
pub fn quick_turn_time_with_target_omega(target_angle: f64, target_omega: f64) -> f64 {
    quick_turn_time_with_target_omega_pure(
        heading(),
        angular_velocity(),
        max_angular_acceleration(),
        target_angle,
        target_omega,
    )
}

/// A pure version of quick_turn_time_with_target_omega that does not call global functions.
pub fn quick_turn_time_with_target_omega_pure(
    heading: f64,
    omega: f64,
    max_ang_accel: f64,
    target_angle: f64,
    target_omega: f64,
) -> f64 {
    let mut x0 = angle_diff(heading, target_angle - target_omega * TICK_LENGTH);
    let mut v0 = omega - target_omega;
    let a = max_ang_accel;
    if a < 1e-6 {
        return f64::INFINITY;
    }

    let a_dec = a * 0.98;
    let k_p = 10.0;
    let theta_trans = a_dec / (k_p * k_p);
    let theta_offset = theta_trans / 2.0;

    // Check if we are already aligned and matched within the 1-tick control window
    let unaccelerated_next_heading = heading + omega * TICK_LENGTH;
    let diff_next = angle_diff(unaccelerated_next_heading, target_angle);
    let speed_diff = (omega - target_omega).abs();
    if diff_next.abs() <= a * TICK_LENGTH * TICK_LENGTH && speed_diff <= a * TICK_LENGTH {
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
        let d =
            (a_dec / (a + a_dec)) * v0 * v0 + (2.0 * a * a_dec / (a + a_dec)) * (x0 - theta_offset);
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

/// Computes the maximum achievable velocity vector in a desired prograde heading direction,
/// given the ship/missile's current kinematic state and the available delta-V (dv).
pub fn max_achievable_velocity(
    kinematic: &crate::physics::KinematicState,
    desired_heading: Vec2,
    available_dv: f64,
) -> Option<Vec2> {
    let v = kinematic.velocity;
    let p = desired_heading;
    let v_dot_p = v.dot(p);
    let v_perp = v - v_dot_p * p;
    let v_perp_len = v_perp.length();

    if available_dv < v_perp_len {
        None
    } else {
        let discriminant = (available_dv * available_dv - v_perp_len * v_perp_len).max(0.0);
        let v_desired_mag = (v_dot_p + discriminant.sqrt()).max(0.0);
        Some(p * v_desired_mag)
    }
}

/// Computes the normalized direction of thrust required to change the current velocity vector
/// toward the target velocity vector. Returns None if they are already matched.
pub fn match_velocity_thrust_heading(
    current_velocity: Vec2,
    target_velocity: Vec2,
) -> Option<Vec2> {
    let dv = target_velocity - current_velocity;
    if dv.length() > 1e-6 {
        Some(dv.normalize())
    } else {
        None
    }
}

#[cfg(test)]
mod control_test;
