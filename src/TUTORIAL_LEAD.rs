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
