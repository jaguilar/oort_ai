use oort_api::prelude::*;
use crate::control::{predict_lead, quick_turn_with_target_omega, AngleTracker};
use crate::radar::{RadarController, DefaultScanSliceGenerator};

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
        let mut rc = RadarController::new();
        // Since the target is outside the default radar distance,
        // we use a narrower search beam width to increase SNR/range.
        // Let's use 0.05 rad (approx. 2.8 degrees) and a max range of 100km.
        rc.slice_generator = Box::new(DefaultScanSliceGenerator::new(0.05, 100000.0));
        // We also use a narrower tracking beam width for keeping locks at range.
        rc.set_tracking_width(0.02);
        // Set the tracking gate radius (uncertainty/clipping window) to 200m.
        rc.set_gate_radius(200.0);

        Ship {
            radar_controller: rc,
            angle_tracker: AngleTracker::new(5.0), // 5 frames time constant
            tracked_bullet: None,
        }
    }

    pub fn tick(&mut self) {
        const BULLET_SPEED: f64 = 1000.0;

        // 1. Update radar scheduler and contact database
        self.radar_controller.update();

        // 2. Aim and shoot at the persistent target (prioritizing targets moving towards us)
        if let Some(target) = self.radar_controller.update_target(position(), velocity()) {
            draw_text!(position() + vec2(0.0, 50.0), rgb(255, 0, 0), "LOCKED ON TARGET!");
            draw_square(target.position, 40.0, rgb(255, 0, 0));

            // Predict optimal lead target angle and time-to-impact
            let mut target_angle_now = None;
            let mut time_to_impact_opt = None;
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
                time_to_impact_opt = Some(time_to_impact);

                // Draw heading and target predictions
                let heading_dir = vec2(heading().cos(), heading().sin());
                draw_line(position(), position() + heading_dir * 1000.0, rgb(0, 255, 255));
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

            let is_close_enough = time_to_impact_opt.map(|t| t < 5.0).unwrap_or(false);

            // Maximize forward acceleration towards target when aligned
            let heading_dir = Vec2::new(heading().cos(), heading().sin());
            let angle_to_target = angle_diff(heading(), target_angle);
            if is_close_enough {
                // Prioritize aiming: do not use boost, and only accelerate forward when very precisely aligned
                if angle_to_target.abs() < 2.0f64.to_radians() {
                    accelerate(heading_dir * max_forward_acceleration());
                } else {
                    // Dampen velocity to help the ship turn and align its trajectory
                    accelerate(-0.5 * velocity());
                }
            } else {
                // Far away: accelerate towards target as quickly as we can, including using boost
                if angle_to_target.abs() < 15.0f64.to_radians() {
                    accelerate(heading_dir * max_forward_acceleration());
                    if angle_to_target.abs() < 5.0f64.to_radians() {
                        activate_ability(Ability::Boost);
                    }
                } else {
                    accelerate(-0.5 * velocity());
                }
            }

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
            // Searching...
            draw_text!(position() + vec2(0.0, 50.0), rgb(0, 150, 255), "Scanning space...");
            
            // Draw a visualization of the scanning beam sweep
            let rh = radar_heading();
            let sweep_dir = Vec2::new(rh.cos(), rh.sin());
            draw_line(position(), position() + sweep_dir * 100000.0, rgb(0, 100, 255));

            // Dampen velocity to remain steady during search
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
