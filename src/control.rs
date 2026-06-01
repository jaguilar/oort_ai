use oort_api::prelude::*;
use crate::radar::{RadarController, Contact, RADIO_PING_SNR};

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

    let difference = angle_diff(heading, target_heading_next - target_omega_next * TICK_LENGTH);
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
        let target_heading_unwrapped = target_heading - 2.0 * std::f64::consts::PI * ((target_heading - target_heading_0_unwrapped) / (2.0 * std::f64::consts::PI)).round();
        
        let r_len_sq = p_rel.dot(p_rel);
        let target_omega = if r_len_sq > 1e-6 {
            let v_rel = target_vel - our_vel + (target_accel - our_accel) * t_align;
            cross(p_rel, v_rel) / r_len_sq
        } else {
            0.0
        };
        
        let t = 0.5 * (t_align + (target_omega - omega) / (s * max_ang_accel));
        let theta_our = heading + omega * t_align + s * max_ang_accel * (2.0 * t * t_align - t * t - 0.5 * t_align * t_align + (t - 0.5 * t_align) * TICK_LENGTH);
        
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
    torque(quick_turn_torque_kinematic(target_pos, target_vel, target_accel, our_accel));
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

/// Telemetry data for a tracked target, transmitted securely over radio.
#[derive(Clone, Copy, Debug)]
pub struct TargetTelemetry {
    pub position: Vec2,
    pub velocity: Vec2,
    pub rssi: f32,
    pub class: Class,
    pub tick: u8,
}

impl TargetTelemetry {
    pub fn serialize(&self) -> [u8; 30] {
        let mut payload = [0u8; 30];
        payload[0..4].copy_from_slice(&(self.position.x as f32).to_le_bytes());
        payload[4..8].copy_from_slice(&(self.position.y as f32).to_le_bytes());
        payload[8..12].copy_from_slice(&(self.velocity.x as f32).to_le_bytes());
        payload[12..16].copy_from_slice(&(self.velocity.y as f32).to_le_bytes());
        payload[16..20].copy_from_slice(&self.rssi.to_le_bytes());
        payload[20] = self.tick;
        payload[21] = match self.class {
            Class::Fighter => 1,
            Class::Frigate => 2,
            Class::Cruiser => 3,
            Class::Missile => 4,
            Class::Torpedo => 5,
            Class::Target => 6,
            _ => 0,
        };
        payload
    }

    pub fn deserialize(payload: &[u8; 30]) -> Self {
        let pos_x = f32::from_le_bytes(payload[0..4].try_into().unwrap()) as f64;
        let pos_y = f32::from_le_bytes(payload[4..8].try_into().unwrap()) as f64;
        let vel_x = f32::from_le_bytes(payload[8..12].try_into().unwrap()) as f64;
        let vel_y = f32::from_le_bytes(payload[12..16].try_into().unwrap()) as f64;
        let rssi = f32::from_le_bytes(payload[16..20].try_into().unwrap());
        let tick = payload[20];
        let class = match payload[21] {
            1 => Class::Fighter,
            2 => Class::Frigate,
            3 => Class::Cruiser,
            4 => Class::Missile,
            5 => Class::Torpedo,
            6 => Class::Target,
            _ => Class::Fighter,
        };
        TargetTelemetry {
            position: vec2(pos_x, pos_y),
            velocity: vec2(vel_x, vel_y),
            rssi,
            class,
            tick,
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
    pub fuel_economy_vc_threshold: f64,
    pub min_search_fuel: f64,
    pub turn_safety_buffer_ticks: f64,

    // State
    pub radar_controller: RadarController,
    pub angle_tracker: AngleTracker,
    pub initial_fuel: f64,
    pub target_id: Option<u32>,
    pub first_detection_tick: Option<u32>,
    pub target_channel: usize,
    pub secure_radio: Option<crate::radio::SecureRadio>,
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
            fuel_economy_vc_threshold: 800.0,
            min_search_fuel: 500.0,
            turn_safety_buffer_ticks: 1.0,

            radar_controller: RadarController::new(),
            angle_tracker: AngleTracker::new(5.0),
            initial_fuel: fuel(),
            target_id: None,
            first_detection_tick: None,
            target_channel: 3,
            secure_radio: None,
        }
    }

    fn target_position_at(&self, target: &Contact, tick: u32, vel_eff: Vec2, accel_eff: Vec2) -> Vec2 {
        let dt = (tick as f64 - target.last_scanned as f64) * TICK_LENGTH;
        target.position + vel_eff * dt + 0.5 * accel_eff * dt * (dt + TICK_LENGTH)
    }

    fn calculate_intercept(&self, target: &Contact) -> (f64, Vec2) {
        let r = target.current_position() - position();
        let r_len = r.length();
        let v_rel = target.current_velocity() - velocity();
        let v_c = if r_len > 1e-6 { -v_rel.dot(r) / r_len } else { 0.0 };
        let t_go = if v_c > 0.0 { r_len / v_c } else { f64::MAX };

        // If we're travelling very slowly compared to the target, then any small amount of velocity
        // will make intercept seem impossibly far away. We only pay attention to the enemy's velocity once we are moving.
        let vel_weight = ((v_c - 100.0) / 300.0).clamp(0.0, 1.0);

        // Similarly, if we're a long way from intercept, the target can make us burn a lot of d_v
        // by juking, when really whether that even matters is unpredictable until we get closer.
        let accel_weight = ((5.0 - t_go)/2.0).clamp(0.0, 1.0);

        let intercept_tick = current_tick() + (t_go / TICK_LENGTH).round() as u32;
        let intercept_point = self.target_position_at(target, intercept_tick, target.velocity * vel_weight, target.acceleration * accel_weight);

        (t_go, intercept_point)
    }

    fn determine_guidance_mode(
        &self,
        has_target: bool,
        is_terminal: bool,
        fuel_economy: bool,
    ) -> &'static str {
        if !has_target {
            if fuel() >= self.min_search_fuel {
                "Search Mode"
            } else {
                "Coast Mode"
            }
        } else if is_terminal {
            "Terminal Turn"
        } else if fuel_economy {
            "Fuel Economy"
        } else {
            "Standard PN Guidance"
        }
    }


    fn check_fuel_economy(&self, v_c: f64, r_len: f64, target_class: Class) -> bool {
        let possible_enemy_dv = if v_c > 0.0 {
            let t_intercept = r_len / v_c;
            let base_dv = t_intercept * target_class.default_stats().max_forward_acceleration;
            let boost_dv = (t_intercept / 10.0).ceil() * 100.0;
            base_dv + boost_dv
        } else {
            0.0
        };
        v_c >= self.fuel_economy_vc_threshold && fuel() < possible_enemy_dv
    }

    pub fn tick(&mut self) {

        if let Some(ref sr) = self.secure_radio {
            // Secure radio mode
            if let Some(payload) = sr.receive() {
                let telemetry = TargetTelemetry::deserialize(&payload);
                debug!("Decoded secure radio ping: pos=({:.1}, {:.1}) vel=({:.1}, {:.1}) rssi={} class={:?}", telemetry.position.x, telemetry.position.y, telemetry.velocity.x, telemetry.velocity.y, telemetry.rssi, telemetry.class);
                let target_id = self.radar_controller.add_radio_ping(telemetry);
                if self.target_id.is_none() {
                    self.target_id = Some(target_id)
                }
            } 

            sr.prepare_receive();
        } else {
            // Standard radio mode
            // 1. Listen on the target radio channel
            select_radio(0);
            set_radio_channel(self.target_channel);

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
                    let telemetry = TargetTelemetry {
                        position: vec2(pos_x, pos_y),
                        velocity: vec2(vel_x, vel_y),
                        rssi: 0.0,
                        class: Class::Fighter,
                        tick: current_tick() as u8,
                    };
                    debug!("Decoded radio float ping on channel {}: pos=({:.1}, {:.1})", self.target_channel, telemetry.position.x, telemetry.position.y);
                    self.radar_controller.add_radio_ping(telemetry);
                }
            }
        }

        // Set current target as high priority in the radar controller
        self.radar_controller.priority_targets = self.target_id.map(|id| vec![id]).unwrap_or_default();
        self.radar_controller.update();
        let contacts = self.radar_controller.contacts();
        let current_t = current_tick();

        // 3. Target selection
        // A target is valid if it is still tracked in the contact list and is a Fighter
        let target_still_valid = if let Some(id) = self.target_id {
            contacts.iter().any(|c| c.id == id && c.class != Class::Missile)
        } else {
            false
        };

        // Filter contacts to only target Class::Fighter
        let fighters: Vec<&Contact> = contacts.iter()
            .filter(|c| c.class != Class::Missile)
            .collect();

        // Set the first detection tick if we've just detected a fighter
        if !fighters.is_empty() && self.first_detection_tick.is_none() {
            self.first_detection_tick = Some(current_t);
        }

        if !target_still_valid {
            // Delay target selection until we've had time for two full scans after first target detection,
            // unless we have a confirmed target from radio telemetry (indicated by snr == RADIO_PING_SNR).
            let has_confirmed_radio_fighter = fighters.iter().any(|f| f.snr == RADIO_PING_SNR);
            let can_lock = if has_confirmed_radio_fighter {
                true
            } else if let Some(first_tick) = self.first_detection_tick {
                current_t - first_tick >= self.target_lock_delay_ticks
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

                // Calculate the anticipated intercept point using the weighted acceleration logic
                let (time_to_intercept, intercept_point) = self.calculate_intercept(target);

                // Lateral acceleration command perpendicular to LOS in the direction of rotation
                let e_perp = vec2(-r.y, r.x) / r_len;
                let a_lateral = self.pn_gain * v_c.max(self.pn_min_vc) * los_rate * e_perp;

                // Forward acceleration with fuel economy check
                let dir = if r_len > 1e-6 {
                    r / r_len
                } else {
                    vec2(heading().cos(), heading().sin())
                };

                // Check if we need to engage fuel economy mode (e.g. if fuel is low relative to possible enemy maneuvers)
                let fuel_economy = self.check_fuel_economy(v_c, r_len, target_class);

                let forward_acc = if fuel_economy {
                    0.0
                } else {
                    max_forward_acceleration()
                };

                let a_total = if !fuel_economy {
                    let r_intercept = intercept_point - position();
                    let dir_intercept = if r_intercept.length() > 1e-6 {
                        r_intercept.normalize()
                    } else {
                        vec2(heading().cos(), heading().sin())
                    };

                    let perp_dir = vec2(-dir_intercept.y, dir_intercept.x);
                    let v_perp = velocity().dot(perp_dir);
                    let alpha = if v_perp.abs() <= 100.0 {
                        (-v_perp / 100.0).asin()
                    } else {
                        -v_perp.signum() * std::f64::consts::FRAC_PI_2
                    };
                    let desired_boost_heading = dir_intercept.angle() + alpha;
                    let acc_dir = vec2(desired_boost_heading.cos(), desired_boost_heading.sin());
                    acc_dir * (max_forward_acceleration() + 100.0)
                } else {
                    a_lateral + dir * forward_acc
                };

                // Turn to point directly at target intercept point when it's time to explode
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
                let mode = self.determine_guidance_mode(true, is_terminal, fuel_economy);
                debug!("Mode: {}", mode);
                debug!("Acc X: {:.2}", a_total.x);
                debug!("Acc Y: {:.2}", a_total.y);
                debug!("Lat Acc X: {:.2}, Y: {:.2}", a_lateral.x, a_lateral.y);
                debug!("Fwd Acc X: {:.2}, Y: {:.2}", (dir * forward_acc).x, (dir * forward_acc).y);

                // Boost should only be used while the target-direction acceleration vector component is greater than 100 m/s^2
                // and the missile is aimed toward the direction it is trying to accelerate in.
                // If that is not the case, actively deactivate boost.
                let target_accel_component = if !fuel_economy {
                    a_total.length()
                } else {
                    a_total.dot(dir)
                };
                let aimed_correctly = if a_total.length() > 0.0 {
                    angle_diff(heading(), a_total.angle()).abs() < 5.0f64.to_radians()
                } else {
                    false
                };

                if !fuel_economy && target_accel_component > 100.0 && aimed_correctly {
                    activate_ability(Ability::Boost);
                } else {
                    deactivate_ability(Ability::Boost);
                }

                // Draw projected intercept point of the currently selected target
                if v_c > 0.0 {
                    draw_diamond(intercept_point, 16.0, rgb(255, 0, 0));
                    draw_line(position(), intercept_point, rgb(255, 0, 0));
                    draw_line(target_pos, intercept_point, rgb(0, 255, 0)); // Draw vector from target to intercept in green
                    draw_text!(intercept_point + vec2(0.0, 20.0), rgb(255, 0, 0), "Intercept: {:.2}s", time_to_intercept);
                }

                // Draw the current position of the currently selected target
                draw_square(target_pos, 20.0, rgb(255, 0, 0));
            }
        } else {
            // No target - burn straight ahead at maximum speed until we find a lock, provided we retain fuel
            let mode = self.determine_guidance_mode(false, false, false);
            let a_cmd = if mode == "Search Mode" {
                let heading_dir = vec2(heading().cos(), heading().sin());
                heading_dir * max_forward_acceleration()
            } else {
                vec2(0.0, 0.0)
            };
            debug!("Mode: {}", mode);
            debug!("Acc X: {:.2}", a_cmd.x);
            debug!("Acc Y: {:.2}", a_cmd.y);

            accelerate(a_cmd);
            deactivate_ability(Ability::Boost);
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
mod control_test;
