use oort_api::prelude::*;

/// Calculates the clamped torque required to turn toward the target angle, taking target angular velocity into account.
pub fn quick_turn_torque_with_target_omega(target_angle: f64, target_omega: f64) -> f64 {
    let difference = angle_diff(heading(), target_angle);
    let omega = angular_velocity();
    let max_ang_accel = max_angular_acceleration();
    
    // Safety buffer: use 98% of max angular acceleration to prevent any overshoot
    let a_dec = max_ang_accel * 0.98;
    let k_p = 10.0;
    
    let theta_trans = a_dec / (k_p * k_p);
    let theta_offset = theta_trans / 2.0;
    
    let omega_target_static = if difference.abs() <= theta_trans {
        k_p * difference
    } else {
        difference.signum() * (2.0 * a_dec * (difference.abs() - theta_offset)).sqrt()
    };
    
    let omega_target = omega_target_static + target_omega;
    let alpha_req = (omega_target - omega) / TICK_LENGTH;
    alpha_req.clamp(-max_ang_accel, max_ang_accel)
}

/// Calculates the clamped torque required to turn toward the target angle without overshooting.
pub fn quick_turn_torque(target_angle: f64) -> f64 {
    quick_turn_torque_with_target_omega(target_angle, 0.0)
}

/// Turn at the maximum possible speed for a given ship that will not overshoot the target angle, taking target angular velocity into account.
pub fn quick_turn_with_target_omega(target_angle: f64, target_omega: f64) {
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

