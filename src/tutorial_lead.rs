use oort_api::prelude::*;
use crate::control::{predict_lead, quick_turn_with_target_omega, AngleTracker};

pub struct Ship {
    prev_target_vel: Option<Vec2>,
    angle_tracker: AngleTracker,
    tracked_bullet: Option<TrackedBullet>,
}

struct TrackedBullet {
    fire_pos: Vec2,
    fire_vel: Vec2,
    fire_dir: Vec2,
    elapsed_time: f64,
}

impl Ship {
    pub fn new() -> Ship {
        Ship {
            prev_target_vel: None,
            angle_tracker: AngleTracker::new(5.0), // 5 frames time constant
            tracked_bullet: None,
        }
    }

    pub fn tick(&mut self) {
        // Zero linear acceleration for this scenario.
        let our_accel = Vec2::new(0.0, 0.0);
        accelerate(our_accel);

        // Track target state locally
        let current_target_vel = target_velocity();
        let target_accel = match self.prev_target_vel {
            Some(prev_vel) => (current_target_vel - prev_vel) / TICK_LENGTH,
            None => Vec2::new(0.0, 0.0),
        };
        self.prev_target_vel = Some(current_target_vel);

        const BULLET_SPEED: f64 = 1000.0;

        // Update and draw tracked bullet if active
        if let Some(ref mut bullet) = self.tracked_bullet {
            // Predict where the bullet is expected to be based on same-turn firing and 20m gun offset
            let predicted_pos = bullet.fire_pos
                + bullet.fire_vel * bullet.elapsed_time
                + (BULLET_SPEED * bullet.elapsed_time + 20.0) * bullet.fire_dir;

            debug!(
                "Tracked bullet at t={:.4}s: predicted_pos={:?}",
                bullet.elapsed_time, predicted_pos
            );

            // Draw equilateral triangle at the predicted position (Red)
            draw_triangle(predicted_pos, 15.0, rgb(255, 0, 0));

            bullet.elapsed_time += TICK_LENGTH;
            if bullet.elapsed_time >= 5.0 {
                self.tracked_bullet = None;
            }
        }

        debug!("--- TICK ---");
        debug!("Our pos: {:?}, vel: {:?}, heading: {:.2} deg", position(), velocity(), heading().to_degrees());
        debug!("Target pos: {:?}, vel: {:?}, accel: {:?}", target(), target_velocity(), target_accel);

        // 1. Predict optimal lead target angle and time-to-impact for the CURRENT tick
        let mut target_angle_now = None;
        if let Some((time_to_impact, lead_dir)) = predict_lead(
            position(),
            velocity(),
            BULLET_SPEED,
            target(),
            target_velocity(),
            target_accel,
        ) {
            let angle = lead_dir.angle();
            target_angle_now = Some(angle);

            // Draw current heading of the ship (Cyan)
            let heading_dir = vec2(heading().cos(), heading().sin());
            draw_line(position(), position() + heading_dir * 1000.0, rgb(0, 255, 255));

            // Draw ray to the predicted location of the enemy (Yellow)
            let p_e = target() + time_to_impact * target_velocity() + 0.5 * target_accel * time_to_impact * (time_to_impact + TICK_LENGTH);
            draw_line(position(), p_e, rgb(255, 255, 0));
        }

        // Determine current target angle (fallback to relative target direction if no lead solution)
        let target_angle = if let Some(angle) = target_angle_now {
            angle
        } else {
            (target() - position()).angle()
        };

        // Update EWMA of the rate of change of the target angle using the tracker helper
        let target_omega = self.angle_tracker.update(target_angle);
        debug!("Target omega EWMA: {:.4} rad/s", target_omega);

        // 2. Apply torque to track target_angle with velocity target_omega
        quick_turn_with_target_omega(target_angle, target_omega);

        // 3. Fire if the weapon is ready and we are aligned with target_angle_now
        if let Some(angle_now) = target_angle_now {
            let diff = angle_diff(heading(), angle_now);
            debug!("Lead diff: {:.2} deg", diff.to_degrees());
            if reload_ticks(0) == 0 && diff.abs() < 0.15f64.to_radians() {
                debug!("FIRED!");
                fire(0);
                if self.tracked_bullet.is_none() {
                    let h = heading();
                    self.tracked_bullet = Some(TrackedBullet {
                        fire_pos: position(),
                        fire_vel: velocity(),
                        fire_dir: Vec2::new(h.cos(), h.sin()),
                        elapsed_time: TICK_LENGTH,
                    });
                }
            }
        }
    }
}
