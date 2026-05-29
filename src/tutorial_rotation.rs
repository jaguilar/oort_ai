use oort_api::prelude::*;
use crate::control::quick_turn;

pub struct Ship;

impl Ship {
    pub fn new() -> Ship {
        Ship
    }

    pub fn tick(&mut self) {
        // Zero acceleration since we don't want to accelerate toward the target.
        accelerate(vec2(0.0, 0.0));

        let target_pos = target();
        let to_target = target_pos - position();
        
        let target_angle = if to_target.length() > 0.0 {
            // General lead prediction formula (incorporating target and ship velocities)
            const BULLET_SPEED: f64 = 1000.0;
            let rel_vel = target_velocity() - velocity();
            
            let a = to_target.dot(to_target);
            let b = to_target.dot(rel_vel);
            let c = rel_vel.dot(rel_vel) - BULLET_SPEED * BULLET_SPEED;
            let discriminant = b * b - a * c;
            
            if discriminant >= 0.0 {
                let k = (-b + discriminant.sqrt()) / a;
                let target_vec = to_target * k + rel_vel;
                if target_vec.length() > 0.0 {
                    target_vec.angle()
                } else {
                    to_target.angle()
                }
            } else {
                to_target.angle()
            }
        } else {
            heading()
        };

        // Turn to target as quickly as possible
        quick_turn(target_angle);

        // Fire when aimed within 0.5 degrees
        let difference = angle_diff(heading(), target_angle);
        if reload_ticks(0) == 0 && difference.abs() < 0.5f64.to_radians() {
            fire(0);
        }
    }
}
