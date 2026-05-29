use oort_api::prelude::*;
use crate::control::{quick_turn_with_target_omega, AngleTracker, MissileGuidance};
use crate::radar::RadarController;

pub struct Ship {
    radar_controller: RadarController,
    angle_tracker: AngleTracker,
    missile_guidance: MissileGuidance,
}

impl Ship {
    pub fn new() -> Ship {
        let rc = RadarController::new();
        let mut mg = MissileGuidance::new();
        // Since tutorial_missile uses channel 0 for fighter-to-missile target communication
        mg.target_channel = 0;

        // Initialize radio for both fighter and missile
        select_radio(0);
        set_radio_channel(0);

        Ship {
            radar_controller: rc,
            angle_tracker: AngleTracker::new(5.0),
            missile_guidance: mg,
        }
    }

    pub fn tick(&mut self) {
        if class() == Class::Missile {
            self.missile_guidance.tick();
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
