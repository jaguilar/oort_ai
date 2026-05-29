use oort_api::prelude::*;
use crate::control::{quick_turn_with_target_omega, AngleTracker, predict_lead};
use crate::radar::{RadarController, Contact};

pub struct Ship {
    radar_controller: RadarController,
    angle_tracker: AngleTracker,
    initial_fuel: f64,
    target_id: Option<u32>,
    first_detection_tick: Option<u32>,
    launch_offset: Option<f64>,
}

impl Ship {
    pub fn new() -> Ship {
        let rc = RadarController::new();

        select_radio(0);
        set_radio_channel(0);

        // For missiles, set the initial radar heading in the direction of the missile's heading.
        if class() == Class::Missile {
            select_radar(0);
            set_radar_heading(heading());
        }

        Ship {
            radar_controller: rc,
            angle_tracker: AngleTracker::new(5.0),
            initial_fuel: fuel(),
            target_id: None,
            first_detection_tick: None,
            launch_offset: None,
        }
    }

    pub fn tick(&mut self) {
        if class() == Class::Missile {
            // Missile behavior
            self.radar_controller.update();

            let contacts = self.radar_controller.contacts();

            // A target is valid if it is still tracked in the contact list and is a Fighter
            let target_still_valid = if let Some(id) = self.target_id {
                contacts.iter().any(|c| c.id == id && c.class == Class::Fighter)
            } else {
                false
            };

            // Filter contacts to only target Class::Fighter
            let fighters: Vec<&Contact> = contacts.iter()
                .filter(|c| c.class == Class::Fighter)
                .collect();

            // Set the first detection tick if we've just detected a fighter
            if !fighters.is_empty() && self.first_detection_tick.is_none() {
                self.first_detection_tick = Some(current_tick());
            }

            if !target_still_valid {
                // Delay target selection until we've had time for two full scans (22 ticks) after first target detection
                let can_lock = if let Some(first_tick) = self.first_detection_tick {
                    current_tick() - first_tick >= 22
                } else {
                    false
                };

                if can_lock && !fighters.is_empty() {
                    // Pick a random fighter instead of the closest one
                    let idx = (rand(0.0, fighters.len() as f64).floor() as usize).min(fighters.len() - 1);
                    self.target_id = Some(fighters[idx].id);
                } else {
                    self.target_id = None;
                }
            }

            if let Some(tid) = self.target_id {
                if let Some(target) = contacts.iter().find(|c| c.id == tid) {
                    let target_pos = target.position;
                    let target_vel = target.velocity;
                    let target_class = target.class;

                    let r = target_pos - position();
                    let r_len = r.length();
                    let v_rel = target_vel - velocity();

                    // 1. Self-destruct proximity check: detonate if within 20m or will be within 5 ticks
                    let next_r = r + v_rel * (5.0 * TICK_LENGTH);
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

                    // Forward acceleration with fuel economy check
                    let dir = if r_len > 1e-6 {
                        r / r_len
                    } else {
                        vec2(heading().cos(), heading().sin())
                    };

                    let expended_fuel = self.initial_fuel - fuel();
                    let possible_enemy_dv = if v_c > 0.0 {
                        let t_intercept = r_len / v_c;
                        let base_dv = t_intercept * target_class.default_stats().max_forward_acceleration;
                        let boost_dv = (t_intercept / 10.0).ceil() * 100.0;
                        base_dv + boost_dv
                    } else {
                        0.0
                    };

                    // Engages if we have expended at least 200 m/s delta v and remaining fuel is low
                    let fuel_economy = expended_fuel >= 200.0 && fuel() < possible_enemy_dv;

                    let forward_acc = if fuel_economy {
                        0.0
                    } else {
                        max_forward_acceleration()
                    };

                    let a_total = a_lateral + dir * forward_acc;

                    // Turn to point directly at target intercept point when it's time to explode (5 ticks before intercept)
                    let time_to_intercept = if v_c > 0.0 { r_len / v_c } else { f64::MAX };
                    let time_until_explosion = (time_to_intercept - 5.0 * TICK_LENGTH).max(0.0);

                    // Heading we need to face at the moment of explosion
                    let position_at_explosion = position() + time_until_explosion * velocity();
                    let target_pos_at_explosion = target_pos 
                        + time_until_explosion * target_vel 
                        + 0.5 * target.acceleration * time_until_explosion * (time_until_explosion + TICK_LENGTH);
                    let explode_heading = (target_pos_at_explosion - position_at_explosion).angle();

                    // Calculate how long we need to turn from the current heading to explode_heading
                    let diff = angle_diff(heading(), explode_heading);
                    let omega = angular_velocity();
                    let a = max_angular_acceleration().max(1.0);

                    let time_to_stop = if omega * diff < 0.0 { omega.abs() / a } else { 0.0 };
                    let angle_to_stop = 0.5 * omega.powi(2) / a;
                    let remaining_angle = (diff.abs() + if omega * diff < 0.0 { angle_to_stop } else { -angle_to_stop }).max(0.0);
                    let time_remaining_turn = 2.0 * (remaining_angle / a).sqrt();
                    let turn_time = time_to_stop + time_remaining_turn;

                    // Add a small safety buffer (e.g. 1 tick) to ensure we finish the turn in time
                    let safety_buffer = 1.0 * TICK_LENGTH;
                    let turn_time_with_buffer = turn_time + safety_buffer;

                    let target_angle = if time_until_explosion <= turn_time_with_buffer {
                        explode_heading
                    } else {
                        a_total.angle()
                    };

                    let target_omega = self.angle_tracker.update(target_angle);
                    quick_turn_with_target_omega(target_angle, target_omega);

                    accelerate(a_total);

                    // Boost to reach target faster, but only if not in fuel economy mode
                    if !fuel_economy {
                        activate_ability(Ability::Boost);
                    }
                }
            } else {
                // No target - burn straight ahead at maximum speed until we find a lock, provided we retain at least 500 m/s of dv
                if fuel() >= 500.0 {
                    let heading_dir = vec2(heading().cos(), heading().sin());
                    accelerate(heading_dir * max_forward_acceleration());
                    activate_ability(Ability::Boost);
                } else {
                    // Coast and decelerate slightly to preserve remaining fuel
                    accelerate(vec2(0.0, 0.0));
                    deactivate_ability(Ability::Boost);
                }
            }
        } else {
            // Fighter behavior
            self.radar_controller.update();

            let contacts = self.radar_controller.contacts();

            // 1. Prioritize incoming missiles with the earliest intercept/impact time
            let mut best_missile_target: Option<(Contact, f64, Vec2)> = None;

            debug!("--- POINT DEFENSE CANDIDATES ---");
            for c in contacts.iter().filter(|c| c.class == Class::Missile) {
                let r = c.position - position();
                let v_rel = c.velocity - velocity();
                let closing = r.dot(v_rel);
                if closing < 0.0 {
                    const BULLET_SPEED: f64 = 1000.0;
                    if let Some((time_to_impact, lead_dir)) = predict_lead(
                        position(),
                        velocity(),
                        BULLET_SPEED,
                        c.position,
                        c.velocity,
                        c.acceleration,
                    ) {
                        let p_e = c.position + time_to_impact * c.velocity + 0.5 * c.acceleration * time_to_impact * (time_to_impact + TICK_LENGTH);
                        let dist_intercept_to_missile = p_e.distance(c.position);
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
                        let dist_a = position().distance(a.position);
                        let dist_b = position().distance(b.position);
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
                        target.position,
                        target.velocity,
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
                        let base_angle = (target.position - position()).angle();
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
                draw_square(target.position, 40.0, rgb(255, 0, 0));

                if let (Some(_lead_dir), Some(t_impact)) = (active_lead_dir, active_time_to_impact) {
                    let p_lead = target.position + t_impact * target.velocity + 0.5 * target.acceleration * t_impact * (t_impact + TICK_LENGTH);
                    draw_square(p_lead, 16.0, rgb(255, 0, 0));
                    draw_line(position(), p_lead, rgb(255, 0, 0));
                    draw_text!(p_lead + vec2(0.0, -20.0), rgb(255, 0, 0), "CHOSEN Intercept: {:.3}s", t_impact);
                }

                let mut target_angle = if let Some(lead_dir) = active_lead_dir {
                    lead_dir.angle()
                } else {
                    (target.position - position()).angle()
                };

                let true_lead_angle = target_angle;

                if !target_is_missile {
                    if let Some(offset) = self.launch_offset {
                        target_angle += offset;
                    }
                }

                let target_omega = self.angle_tracker.update(target_angle);
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
