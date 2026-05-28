use oort_api::prelude::*;
use crate::control::{quick_turn_with_target_omega, AngleTracker};
use crate::radar::RadarController;

pub struct Ship {
    radar_controller: RadarController,
    angle_tracker: AngleTracker,
}

impl Ship {
    pub fn new() -> Ship {
        let rc = RadarController::new();

        // Initialize radio for both fighter and missile
        select_radio(0);
        set_radio_channel(0);

        Ship {
            radar_controller: rc,
            angle_tracker: AngleTracker::new(5.0),
        }
    }

    pub fn tick(&mut self) {
        if class() == Class::Missile {
            // Missile behavior
            select_radio(0);
            set_radio_channel(0);

            if let Some(msg) = receive() {
                let target_pos = vec2(msg[0], msg[1]);
                let target_vel = vec2(msg[2], msg[3]);

                let r = target_pos - position();
                let r_len = r.length();
                let v_rel = target_vel - velocity();

                // 1. Self-destruct proximity check: detonate if within 20m or will be within 2 ticks
                let next_r = r + v_rel * (2.0 * TICK_LENGTH);
                if r_len < 20.0 || next_r.length() < 20.0 {
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

                // Lateral acceleration command perpendicular to LOS in the direction of rotation
                let e_perp = vec2(-r.y, r.x) / r_len;
                let n = 4.0;
                let a_lateral = n * v_c.max(100.0) * los_rate * e_perp;

                // Forward acceleration towards the target
                let dir = if r_len > 1e-6 {
                    r / r_len
                } else {
                    vec2(heading().cos(), heading().sin())
                };
                let a_total = a_lateral + dir * max_forward_acceleration();

                // Rotate ship to face the commanded acceleration vector to maximize thrust efficiency.
                // Incorporate the angular velocity of the acceleration vector target.
                let target_angle = a_total.angle();
                let target_omega = self.angle_tracker.update(target_angle);
                quick_turn_with_target_omega(target_angle, target_omega);

                accelerate(a_total);

                // Boost to reach target faster
                activate_ability(Ability::Boost);
            }
        } else {
            // Fighter behavior
            self.radar_controller.update();

            if let Some(target) = self.radar_controller.update_target(position(), velocity()) {
                // Draw locked target indicator
                draw_text!(position() + vec2(0.0, 50.0), rgb(255, 0, 0), "TARGET LOCKED!");
                draw_square(target.position, 40.0, rgb(255, 0, 0));

                // 1. Broadcast target position and velocity on channel 0
                select_radio(0);
                set_radio_channel(0);
                send([target.position.x, target.position.y, target.velocity.x, target.velocity.y]);

                // 2. Steer and run toward the enemy
                let target_angle = (target.position - position()).angle();
                let target_omega = self.angle_tracker.update(target_angle);
                quick_turn_with_target_omega(target_angle, target_omega);

                let heading_dir = vec2(heading().cos(), heading().sin());
                let angle_to_target = angle_diff(heading(), target_angle);

                // Run towards the enemy with max forward thrust and boost if aligned
                if angle_to_target.abs() < 15.0f64.to_radians() {
                    accelerate(heading_dir * max_forward_acceleration());
                    if angle_to_target.abs() < 5.0f64.to_radians() {
                        activate_ability(Ability::Boost);
                    }
                } else {
                    // Turn to align
                    accelerate(-0.5 * velocity());
                }

                // 3. Fire missiles on cooldown, but only if generally pointing at the target
                if reload_ticks(1) == 0 && angle_to_target.abs() < 10.0f64.to_radians() {
                    fire(1);
                }
            } else {
                // Scanning...
                draw_text!(position() + vec2(0.0, 50.0), rgb(0, 150, 255), "Scanning for target...");
                let rh = radar_heading();
                let sweep_dir = vec2(rh.cos(), rh.sin());
                draw_line(position(), position() + sweep_dir * 100000.0, rgb(0, 100, 255));
                accelerate(-0.5 * velocity());
            }
        }
    }
}
