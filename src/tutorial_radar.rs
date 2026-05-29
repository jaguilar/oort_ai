use oort_api::prelude::*;
use crate::control::{predict_lead, quick_turn_with_target_omega, AngleTracker};
use crate::radar::RadarController;

pub struct Ship {
    radar_controller: RadarController,
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
            radar_controller: RadarController::new(),
            angle_tracker: AngleTracker::new(5.0), // 5 frames time constant
            tracked_bullet: None,
        }
    }

    pub fn tick(&mut self) {
        const BULLET_SPEED: f64 = 1000.0;

        // 1. Update radar scheduler and contact database
        self.radar_controller.update();

        // 2. Station keeping: hold station at the geometric center of all active contacts
        let center = self.radar_controller.geometric_center(position());

        // Draw station keeping target position
        draw_square(center, 25.0, rgb(0, 255, 0));
        draw_line(position(), center, rgb(0, 255, 0));
        draw_text!(center + vec2(0.0, 40.0), rgb(0, 255, 0), "Geometric Center");

        // PD Controller with increased damping to prevent overshoot
        let error = center - position();
        accelerate(0.1 * error - 0.6 * velocity());

        // 3. Aim and shoot at the persistent target (prioritizing targets moving towards us)
        if let Some(target) = self.radar_controller.update_target(position(), velocity()) {
            // Predict optimal lead target angle and time-to-impact
            let mut target_angle_now = None;
            if let Some((time_to_impact, lead_dir)) = predict_lead(
                position(),
                velocity(),
                BULLET_SPEED,
                target.position,
                target.velocity,
                target.acceleration,
            ) {
                let angle = lead_dir.angle();
                target_angle_now = Some(angle);

                // Draw current heading of the ship (Cyan)
                let heading_dir = vec2(heading().cos(), heading().sin());
                draw_line(position(), position() + heading_dir * 1000.0, rgb(0, 255, 255));

                // Draw ray to the predicted location of the enemy (Yellow)
                let p_e = target.position + time_to_impact * target.velocity + 0.5 * target.acceleration * time_to_impact * (time_to_impact + TICK_LENGTH);
                draw_line(position(), p_e, rgb(255, 255, 0));
            }

            // Fallback if no lead solution
            let target_angle = if let Some(angle) = target_angle_now {
                angle
            } else {
                (target.position - position()).angle()
            };

            let target_omega = self.angle_tracker.update(target_angle);
            quick_turn_with_target_omega(target_angle, target_omega);

            // Fire if weapon is ready and aligned
            if let Some(angle_now) = target_angle_now {
                let diff = angle_diff(heading(), angle_now);
                if reload_ticks(0) == 0 && diff.abs() < 0.15f64.to_radians() {
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

        // 4. Update and draw tracked bullet visualization
        if let Some(ref mut bullet) = self.tracked_bullet {
            let predicted_pos = bullet.fire_pos
                + bullet.fire_vel * bullet.elapsed_time
                + (BULLET_SPEED * bullet.elapsed_time + 20.0) * bullet.fire_dir;

            draw_triangle(predicted_pos, 15.0, rgb(255, 0, 0));

            bullet.elapsed_time += TICK_LENGTH;
            if bullet.elapsed_time >= 5.0 {
                self.tracked_bullet = None;
            }
        }
    }
}
