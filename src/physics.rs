use oort_api::prelude::*;

#[derive(Clone, Debug)]
pub struct KinematicState {
    pub class: Class,
    pub position: Vec2,
    pub velocity: Vec2,
    pub acceleration: Vec2,
    pub last_scanned: u32,
    pub heading: Option<f64>,
    pub angular_velocity: Option<f64>,
}

impl KinematicState {
    pub fn new(class: Class, position: Vec2, velocity: Vec2, acceleration: Vec2, last_scanned: u32) -> Self {
        Self {
            class,
            position,
            velocity,
            acceleration,
            last_scanned,
            heading: None,
            angular_velocity: None,
        }
    }

    pub fn self_state() -> Self {
        Self {
            class: class(),
            position: position(),
            velocity: velocity(),
            acceleration: Vec2::new(0.0, 0.0),
            last_scanned: current_tick(),
            heading: Some(heading()),
            angular_velocity: Some(angular_velocity()),
        }
    }

    pub fn position_at(&self, tick: u32) -> Vec2 {
        let dt = (tick - self.last_scanned) as f64 * TICK_LENGTH;
        self.position + self.velocity * dt + 0.5 * self.acceleration * dt * (dt + TICK_LENGTH)
    }

    pub fn velocity_at(&self, tick: u32) -> Vec2 {
        let dt = (tick - self.last_scanned) as f64 * TICK_LENGTH;
        self.velocity + self.acceleration * dt
    }
}

pub fn max_acceleration_over_time(class: Class, t_go: f64) -> f64 {
    let stats = class.default_stats();
    let base_accel = stats.max_forward_acceleration
        .max(stats.max_backward_acceleration)
        .max(stats.max_lateral_acceleration);
    let boost_dv = if class == Class::Fighter || class == Class::Missile {
        (t_go / 10.0).ceil() * 100.0
    } else {
        0.0
    };
    base_accel + boost_dv / t_go.max(1e-6)
}

