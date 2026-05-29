use oort_api::prelude::*;
use crate::control::{predict_lead, quick_turn_with_target_omega, AngleTracker, TargetTracker};

pub struct Ship {
    target_tracker: TargetTracker,
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
        // Select radio 0 and tune to channel 2
        select_radio(0);
        set_radio_channel(2);

        Ship {
            target_tracker: TargetTracker::new(),
            angle_tracker: AngleTracker::new(5.0), // 5 frames time constant
            tracked_bullet: None,
        }
    }

    pub fn tick(&mut self) {
        const BULLET_SPEED: f64 = 1000.0;

        // Ensure radio 0 is tuned to channel 2
        select_radio(0);
        set_radio_channel(2);

        // 1. Read the enemy's position and velocity from the radio signal on channel 2
        if let Some(msg) = receive() {
            let enemy_pos = vec2(msg[0], msg[1]);
            let enemy_vel = vec2(msg[2], msg[3]);
            self.target_tracker.update(current_tick(), enemy_pos, enemy_vel);
        }

        // 2. Aim, steer and shoot if we have located the target
        if self.target_tracker.last_seen_tick().is_some() {
            let target_pos = self.target_tracker.position();
            let target_vel = self.target_tracker.velocity();
            let target_accel = self.target_tracker.acceleration();

            // Draw target visualization
            draw_square(target_pos, 25.0, rgb(255, 0, 0));
            draw_text!(target_pos + vec2(0.0, 40.0), rgb(255, 0, 0), "Radio Target");

            // Follow target: PD controller targeting the enemy position while matching their velocity
            let pos_error = target_pos - position();
            let vel_error = target_vel - velocity();
            accelerate(0.2 * pos_error + 0.8 * vel_error);

            // Predict optimal lead target angle and time-to-impact
            let mut target_angle_now = None;
            if let Some((time_to_impact, lead_dir)) = predict_lead(
                position(),
                velocity(),
                BULLET_SPEED,
                target_pos,
                target_vel,
                target_accel,
            ) {
                let angle = lead_dir.angle();
                target_angle_now = Some(angle);

                // Draw current heading of the ship (Cyan)
                let heading_dir = vec2(heading().cos(), heading().sin());
                draw_line(position(), position() + heading_dir * 1000.0, rgb(0, 255, 255));

                // Draw ray to the predicted location of the enemy (Yellow)
                let p_e = target_pos + time_to_impact * target_vel + 0.5 * target_accel * time_to_impact * (time_to_impact + TICK_LENGTH);
                draw_line(position(), p_e, rgb(255, 255, 0));
            }

            // Fallback if no lead solution
            let target_angle = if let Some(angle) = target_angle_now {
                angle
            } else {
                (target_pos - position()).angle()
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
        } else {
            // Keep still / search mode (should not happen if radio signal is broadcast each tick)
            accelerate(-0.5 * velocity());
        }

        // 3. Update and draw tracked bullet visualization
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
