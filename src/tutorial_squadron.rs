use oort_api::prelude::*;
use crate::control::{quick_turn_with_target_omega, predict_lead};
use crate::missile::MissileGuidance;
use crate::radar::Contact;

pub struct Ship {
    missile_guidance: MissileGuidance,
    launch_offset: Option<f64>,
}

impl Ship {
    pub fn new() -> Ship {
        Ship {
            missile_guidance: MissileGuidance::new(),
            launch_offset: None,
        }
    }

    pub fn tick(&mut self) {
        if class() == Class::Missile {
            self.missile_guidance.tick();
        } else {
            // Fighter behavior
            self.missile_guidance.radar_controller.update();

            let contacts = self.missile_guidance.radar_controller.contacts();

            // 1. Prioritize incoming missiles with the earliest intercept/impact time
            let mut best_missile_target: Option<(Contact, f64, Vec2)> = None;

            debug!("--- POINT DEFENSE CANDIDATES ---");
            for c in contacts.iter().filter(|c| c.class == Class::Missile) {
                let r = c.current_position() - position();
                let v_rel = c.current_velocity() - velocity();
                let closing = r.dot(v_rel);
                if closing < 0.0 {
                    const BULLET_SPEED: f64 = 1000.0;
                    if let Some((time_to_impact, lead_dir)) = predict_lead(
                        position(),
                        velocity(),
                        BULLET_SPEED,
                        c.current_position(),
                        c.current_velocity(),
                        c.acceleration,
                    ) {
                        let p_e = c.position_at(current_tick() + (time_to_impact / TICK_LENGTH).round() as u32);
                        let dist_intercept_to_missile = p_e.distance(c.current_position());
                        let dist_fighter_to_missile = r.length();

                        if dist_intercept_to_missile > dist_fighter_to_missile {
                            debug!("Missile ID {}: DROPPED (intercept dist {:.1}m > missile dist {:.1}m)", c.id, dist_intercept_to_missile, dist_fighter_to_missile);
                            continue;
                        }

                        debug!("Missile ID {}: closing={:.1}m/s, t_intercept={:.3}s, dist={:.1}m, p_e={:?}", c.id, -closing / r.length(), time_to_impact, r.length(), p_e);
                        draw_diamond(p_e, 12.0, rgb(255, 165, 0));
                        draw_text!(p_e + vec2(0.0, 20.0), rgb(255, 165, 0), "PD Sol: {:.3}s", time_to_impact);

                        if best_missile_target.is_none() || time_to_impact < best_missile_target.as_ref().unwrap().1 {
                            best_missile_target = Some((c.clone(), time_to_impact, lead_dir));
                        }
                    } else {
                        debug!("Missile ID {}: approaching but NO lead solution!", c.id);
                    }
                } else {
                    debug!("Missile ID {}: IGNORED (passed, closing={:.1})", c.id, -closing / r.length());
                }
            }

            let mut active_target = None;
            let mut active_lead_dir = None;
            let mut active_time_to_impact = None;

            if let Some((target, time_to_impact, lead_dir)) = best_missile_target {
                active_target = Some(target);
                active_lead_dir = Some(lead_dir);
                active_time_to_impact = Some(time_to_impact);
            } else {
                // Fallback to the closest fighter target
                let fighter_target = contacts.iter()
                    .filter(|c| c.class == Class::Fighter)
                    .min_by(|a, b| {
                        let dist_a = position().distance(a.current_position());
                        let dist_b = position().distance(b.current_position());
                        dist_a.partial_cmp(&dist_b).unwrap_or(std::cmp::Ordering::Equal)
                    })
                    .cloned();

                if let Some(target) = fighter_target {
                    active_target = Some(target.clone());
                    const BULLET_SPEED: f64 = 1000.0;
                    if let Some((time_to_impact, lead_dir)) = predict_lead(
                        position(),
                        velocity(),
                        BULLET_SPEED,
                        target.current_position(),
                        target.current_velocity(),
                        target.acceleration,
                    ) {
                        active_lead_dir = Some(lead_dir);
                        active_time_to_impact = Some(time_to_impact);
                    }
                }
            }

            // Determine if the active target is a missile
            let mut target_is_missile = false;
            if let Some(ref target) = active_target {
                if target.class == Class::Missile {
                    target_is_missile = true;
                }
            }

            if target_is_missile {
                self.launch_offset = None;
                if reload_ticks(1) == 0 {
                    fire(1);
                }
            } else {
                if reload_ticks(1) == 0 {
                    if self.launch_offset.is_none() {
                        self.launch_offset = Some(rand(-12.5f64.to_radians(), 12.5f64.to_radians()));
                    }
                }
                if let Some(offset) = self.launch_offset {
                    if let Some(ref target) = active_target {
                        let base_angle = (target.current_position() - position()).angle();
                        let target_offset_angle = base_angle + offset;
                        if angle_diff(heading(), target_offset_angle).abs() < 4.0f64.to_radians() {
                            fire(1);
                            self.launch_offset = None;
                        }
                    }
                }
            }

            if let Some(target) = active_target {
                let target_class_str = if target.class == Class::Missile { "MISSILE" } else { "FIGHTER" };
                draw_text!(position() + vec2(0.0, 50.0), rgb(255, 0, 0), "TARGET LOCKED: {}", target_class_str);
                draw_square(target.current_position(), 40.0, rgb(255, 0, 0));

                if let (Some(_lead_dir), Some(t_impact)) = (active_lead_dir, active_time_to_impact) {
                    let p_lead = target.position_at(current_tick() + (t_impact / TICK_LENGTH).round() as u32);
                    draw_square(p_lead, 16.0, rgb(255, 0, 0));
                    draw_line(position(), p_lead, rgb(255, 0, 0));
                    draw_text!(p_lead + vec2(0.0, -20.0), rgb(255, 0, 0), "CHOSEN Intercept: {:.3}s", t_impact);
                }

                let mut target_angle = if let Some(lead_dir) = active_lead_dir {
                    lead_dir.angle()
                } else {
                    (target.current_position() - position()).angle()
                };

                let true_lead_angle = target_angle;

                if !target_is_missile {
                    if let Some(offset) = self.launch_offset {
                        target_angle += offset;
                    }
                }

                let target_omega = self.missile_guidance.angle_tracker.update(target_angle);
                quick_turn_with_target_omega(target_angle, target_omega);

                let heading_dir = vec2(heading().cos(), heading().sin());
                let angle_to_target = angle_diff(heading(), target_angle);

                // Run towards the active target direction
                if angle_to_target.abs() < 15.0f64.to_radians() {
                    accelerate(heading_dir * max_forward_acceleration());
                    if angle_to_target.abs() < 5.0f64.to_radians() {
                        activate_ability(Ability::Boost);
                    }
                } else {
                    // Turn to align
                    accelerate(-0.5 * velocity());
                }

                // Fire gun (slot 0) if the weapon is ready and we are aligned with target's true lead angle
                let angle_to_true_lead = angle_diff(heading(), true_lead_angle);
                if reload_ticks(0) == 0 && angle_to_true_lead.abs() < 2.0f64.to_radians() {
                    fire(0);
                }
            } else {
                // Scanning...
                self.launch_offset = None;
                draw_text!(position() + vec2(0.0, 50.0), rgb(0, 150, 255), "Scanning for target...");
                let rh = radar_heading();
                let sweep_dir = vec2(rh.cos(), rh.sin());
                draw_line(position(), position() + sweep_dir * 100000.0, rgb(0, 100, 255));
                accelerate(-0.5 * velocity());
            }
        }
    }
}
