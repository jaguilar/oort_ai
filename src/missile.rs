use oort_api::prelude::*;
use crate::radar::{RadarController, Contact, RADIO_PING_SNR};
use crate::control::{optimal_turn_torque, AngleTracker, newton_solve};
use crate::physics::KinematicState;
use crate::aim::AimAt;

/// Telemetry data for a tracked target, transmitted securely over radio.
#[derive(Clone, Copy, Debug)]
pub struct TargetTelemetry {
    pub position: Vec2,
    pub velocity: Vec2,
    pub rssi: f32,
    pub class: Class,
    pub tick: u8,
}

impl TargetTelemetry {
    pub fn serialize(&self) -> [u8; 30] {
        let mut payload = [0u8; 30];
        payload[0..4].copy_from_slice(&(self.position.x as f32).to_le_bytes());
        payload[4..8].copy_from_slice(&(self.position.y as f32).to_le_bytes());
        payload[8..12].copy_from_slice(&(self.velocity.x as f32).to_le_bytes());
        payload[12..16].copy_from_slice(&(self.velocity.y as f32).to_le_bytes());
        payload[16..20].copy_from_slice(&self.rssi.to_le_bytes());
        payload[20] = self.tick;
        payload[21] = match self.class {
            Class::Fighter => 1,
            Class::Frigate => 2,
            Class::Cruiser => 3,
            Class::Missile => 4,
            Class::Torpedo => 5,
            Class::Target => 6,
            _ => 0,
        };
        payload
    }

    pub fn deserialize(payload: &[u8; 30]) -> Self {
        let pos_x = f32::from_le_bytes(payload[0..4].try_into().unwrap()) as f64;
        let pos_y = f32::from_le_bytes(payload[4..8].try_into().unwrap()) as f64;
        let vel_x = f32::from_le_bytes(payload[8..12].try_into().unwrap()) as f64;
        let vel_y = f32::from_le_bytes(payload[12..16].try_into().unwrap()) as f64;
        let rssi = f32::from_le_bytes(payload[16..20].try_into().unwrap());
        let tick = payload[20];
        let class = match payload[21] {
            1 => Class::Fighter,
            2 => Class::Frigate,
            3 => Class::Cruiser,
            4 => Class::Missile,
            5 => Class::Torpedo,
            6 => Class::Target,
            _ => Class::Fighter,
        };
        TargetTelemetry {
            position: vec2(pos_x, pos_y),
            velocity: vec2(vel_x, vel_y),
            rssi,
            class,
            tick,
        }
    }
}

/// Coordinates firing missiles and transmitting target updates to them.
pub struct MissileRadioSender {
    pub delay_missile_contact: Option<u32>,
    pub missile_radio: crate::radio::SecureRadio,
}

impl MissileRadioSender {
    pub fn new(missile_radio: crate::radio::SecureRadio) -> Self {
        Self {
            delay_missile_contact: None,
            missile_radio,
        }
    }

    pub fn fire_missile(&mut self, weapon_id: usize, target_id: u32) {
        fire(weapon_id);
        self.delay_missile_contact = Some(target_id);
    }

    pub fn send_missile_contact(&mut self, contacts: &[Contact]) {
        if let Some(cid) = self.delay_missile_contact {
            let contact = contacts.iter().find(|&c| c.id == cid);
            if let Some(contact) = contact {
                let telemetry = TargetTelemetry {
                    position: contact.current_position(),
                    velocity: contact.current_velocity(),
                    rssi: contact.rssi as f32,
                    class: contact.class,
                    tick: current_tick() as u8,
                };
                self.missile_radio.transmit(telemetry.serialize());
                debug!("Sent location of {} to missiles", contact.id);
            }
            self.delay_missile_contact = None;
        }
    }
}

/// Missile guidance system encapsulating radar scanning, target selection,
/// proportional navigation, fuel economy, and terminal orientation control.
pub struct MissileGuidance {
    // Adjustable constant parameters
    pub proximity_dist: f64,
    pub proximity_ticks: f64,
    pub pn_gain: f64,
    pub pn_min_vc: f64,
    pub target_lock_delay_ticks: u32,
    pub fuel_economy_vc_threshold: f64,
    pub min_search_fuel: f64,
    pub turn_safety_buffer_ticks: f64,

    // State
    pub radar_controller: RadarController,
    pub angle_tracker: AngleTracker,
    pub initial_fuel: f64,
    pub target_id: Option<u32>,
    pub first_detection_tick: Option<u32>,
    pub target_channel: usize,
    pub secure_radio: Option<crate::radio::SecureRadio>,
}

impl MissileGuidance {
    pub fn new() -> Self {
        // Initial setup for the missile's radar and radio
        select_radio(0);
        set_radio_channel(3);

        if class() == Class::Missile {
            select_radar(0);
            set_radar_heading(heading());
        }

        Self {
            proximity_dist: 20.0,
            proximity_ticks: 5.0,
            pn_gain: 4.0,
            pn_min_vc: 100.0,
            target_lock_delay_ticks: 22,
            fuel_economy_vc_threshold: 800.0,
            min_search_fuel: 500.0,
            turn_safety_buffer_ticks: 1.0,

            radar_controller: RadarController::new(),
            angle_tracker: AngleTracker::new(5.0),
            initial_fuel: fuel(),
            target_id: None,
            first_detection_tick: None,
            target_channel: 3,
            secure_radio: None,
        }
    }

    fn calculate_intercept(&self, target: &Contact) -> (f64, Vec2) {
        MissileAimer::calculate_intercept(&target.kinematic, position(), velocity(), current_tick())
    }

    fn determine_guidance_mode(
        &self,
        has_target: bool,
        is_terminal: bool,
        fuel_economy: bool,
    ) -> &'static str {
        if !has_target {
            if fuel() >= self.min_search_fuel {
                "Search Mode"
            } else {
                "Coast Mode"
            }
        } else if is_terminal {
            "Terminal Turn"
        } else if fuel_economy {
            "Fuel Economy"
        } else {
            "Standard PN Guidance"
        }
    }

    fn check_fuel_economy(&self, v_c: f64, r_len: f64, target_class: Class) -> bool {
        let possible_enemy_dv = if v_c > 0.0 {
            let t_intercept = r_len / v_c;
            let base_dv = t_intercept * target_class.default_stats().max_forward_acceleration;
            let boost_dv = (t_intercept / 10.0).ceil() * 100.0;
            base_dv + boost_dv
        } else {
            0.0
        };
        v_c >= self.fuel_economy_vc_threshold && fuel() < possible_enemy_dv
    }

    pub fn tick(&mut self) {
        if let Some(ref sr) = self.secure_radio {
            // Secure radio mode
            if let Some(payload) = sr.receive() {
                let telemetry = TargetTelemetry::deserialize(&payload);
                debug!("Decoded secure radio ping: pos=({:.1}, {:.1}) vel=({:.1}, {:.1}) rssi={} class={:?}", telemetry.position.x, telemetry.position.y, telemetry.velocity.x, telemetry.velocity.y, telemetry.rssi, telemetry.class);
                let target_id = self.radar_controller.add_radio_ping(telemetry);
                if self.target_id.is_none() {
                    self.target_id = Some(target_id)
                }
            } 

            sr.prepare_receive();
        } else {
            // Standard radio mode
            // 1. Listen on the target radio channel
            select_radio(0);
            set_radio_channel(self.target_channel);

            // Try standard float message first ([f64; 4]: pos_x, pos_y, vel_x, vel_y)
            if let Some(msg) = receive() {
                let pos_x = msg[0];
                let pos_y = msg[1];
                let vel_x = msg[2];
                let vel_y = msg[3];

                // Mismatched byte representation when interpreted as f64 yields astronomical values or NaNs.
                // Check that the numbers are finite and lie within reasonable limits to distinguish formats.
                if pos_x.is_finite() && pos_y.is_finite() && vel_x.is_finite() && vel_y.is_finite()
                    && pos_x.abs() < 100_000.0 && pos_y.abs() < 100_000.0
                    && vel_x.abs() < 10_000.0 && vel_y.abs() < 10_000.0
                {
                    let telemetry = TargetTelemetry {
                        position: vec2(pos_x, pos_y),
                        velocity: vec2(vel_x, vel_y),
                        rssi: 0.0,
                        class: Class::Fighter,
                        tick: current_tick() as u8,
                    };
                    debug!("Decoded radio float ping on channel {}: pos=({:.1}, {:.1})", self.target_channel, telemetry.position.x, telemetry.position.y);
                    self.radar_controller.add_radio_ping(telemetry);
                }
            }
        }

        // Set current target as high priority in the radar controller
        self.radar_controller.priority_targets = self.target_id.map(|id| vec![id]).unwrap_or_default();
        self.radar_controller.update();
        let contacts = self.radar_controller.contacts();
        let current_t = current_tick();

        // 3. Target selection
        // A target is valid if it is still tracked in the contact list and is a Fighter
        let target_still_valid = if let Some(id) = self.target_id {
            contacts.iter().any(|c| c.id == id && c.class != Class::Missile)
        } else {
            false
        };

        // Filter contacts to only target Class::Fighter
        let fighters: Vec<&Contact> = contacts.iter()
            .filter(|c| c.class != Class::Missile)
            .collect();

        // Set the first detection tick if we've just detected a fighter
        if !fighters.is_empty() && self.first_detection_tick.is_none() {
            self.first_detection_tick = Some(current_t);
        }

        if !target_still_valid {
            // Delay target selection until we've had time for two full scans after first target detection,
            // unless we have a confirmed target from radio telemetry (indicated by snr == RADIO_PING_SNR).
            let has_confirmed_radio_fighter = fighters.iter().any(|f| f.snr == RADIO_PING_SNR);
            let can_lock = if has_confirmed_radio_fighter {
                true
            } else if let Some(first_tick) = self.first_detection_tick {
                current_t - first_tick >= self.target_lock_delay_ticks
            } else {
                false
            };

            if can_lock && !fighters.is_empty() {
                // Pick a random fighter instead of the closest one
                let idx = (rand(0.0, fighters.len() as f64).floor() as usize).min(fighters.len() - 1);
                let new_id = fighters[idx].id;
                if let Some(old_id) = self.target_id {
                    debug!("Ceasing targeting of target {} because it is no longer valid (not in contacts list or not a fighter); locking onto new target {}", old_id, new_id);
                }
                self.target_id = Some(new_id);
            } else {
                if let Some(old_id) = self.target_id {
                    debug!("Ceasing targeting of target {} because it is no longer valid (not in contacts list or not a fighter) and no new target lock could be acquired", old_id);
                }
                self.target_id = None;
            }
        }

        if let Some(tid) = self.target_id {
            if let Some(target) = contacts.iter().find(|c| c.id == tid) {
                let target_pos = target.current_position();
                let target_vel = target.current_velocity();
                let target_class = target.class;

                let r = target_pos - position();
                let r_len = r.length();
                let v_rel = target_vel - velocity();

                // 1. Self-destruct proximity check: detonate if within target proximity or will be soon
                let next_r = r + v_rel * (self.proximity_ticks * TICK_LENGTH);
                if r_len < self.proximity_dist || next_r.length() < self.proximity_dist {
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

                // Calculate the anticipated intercept point using the weighted acceleration logic
                let (time_to_intercept, intercept_point) = self.calculate_intercept(target);

                // Lateral acceleration command perpendicular to LOS in the direction of rotation
                let e_perp = vec2(-r.y, r.x) / r_len;
                let a_lateral = self.pn_gain * v_c.max(self.pn_min_vc) * los_rate * e_perp;

                // Forward acceleration with fuel economy check
                let dir = if r_len > 1e-6 {
                    r / r_len
                } else {
                    vec2(heading().cos(), heading().sin())
                };

                // Check if we need to engage fuel economy mode (e.g. if fuel is low relative to possible enemy maneuvers)
                let fuel_economy = self.check_fuel_economy(v_c, r_len, target_class);

                let forward_acc = if fuel_economy {
                    0.0
                } else {
                    max_forward_acceleration()
                };

                let a_total = if !fuel_economy {
                    let r_intercept = intercept_point - position();
                    let dir_intercept = if r_intercept.length() > 1e-6 {
                        r_intercept.normalize()
                    } else {
                        vec2(heading().cos(), heading().sin())
                    };

                    let perp_dir = vec2(-dir_intercept.y, dir_intercept.x);
                    let v_perp = velocity().dot(perp_dir);
                    let alpha = if v_perp.abs() <= 100.0 {
                        (-v_perp / 100.0).asin()
                    } else {
                        -v_perp.signum() * std::f64::consts::FRAC_PI_2
                    };
                    let desired_boost_heading = dir_intercept.angle() + alpha;
                    let acc_dir = vec2(desired_boost_heading.cos(), desired_boost_heading.sin());
                    acc_dir * (max_forward_acceleration() + 100.0)
                } else {
                    a_lateral + dir * forward_acc
                };

                // Turn to point directly at target intercept point when it's time to explode
                let time_until_explosion = (time_to_intercept - self.proximity_ticks * TICK_LENGTH).max(0.0);

                // Heading we need to face at the moment of explosion
                let position_at_explosion = position() + time_until_explosion * velocity();
                let target_pos_at_explosion = target.position_at(current_tick() + (time_until_explosion / TICK_LENGTH).round() as u32);
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

                // Add a small safety buffer to ensure we finish the turn in time
                let safety_buffer = self.turn_safety_buffer_ticks * TICK_LENGTH;
                let turn_time_with_buffer = turn_time + safety_buffer;

                let torque_cmd = if time_until_explosion <= turn_time_with_buffer {
                    optimal_turn_torque(position(), velocity(), target_pos_at_explosion, None)
                } else {
                    let target_pos_aim = position() + a_total;
                    optimal_turn_torque(position(), velocity(), target_pos_aim, Some(velocity()))
                };
                torque(torque_cmd);

                accelerate(a_total);

                // Print guidance mode and acceleration components
                let is_terminal = time_until_explosion <= turn_time_with_buffer;
                let mode = self.determine_guidance_mode(true, is_terminal, fuel_economy);
                debug!("Mode: {}", mode);
                debug!("Acc X: {:.2}", a_total.x);
                debug!("Acc Y: {:.2}", a_total.y);
                debug!("Lat Acc X: {:.2}, Y: {:.2}", a_lateral.x, a_lateral.y);
                debug!("Fwd Acc X: {:.2}, Y: {:.2}", (dir * forward_acc).x, (dir * forward_acc).y);

                // Boost should only be used while the target-direction acceleration vector component is greater than 100 m/s^2
                // and the missile is aimed toward the direction it is trying to accelerate in.
                // If that is not the case, actively deactivate boost.
                let target_accel_component = if !fuel_economy {
                    a_total.length()
                } else {
                    a_total.dot(dir)
                };
                let aimed_correctly = if a_total.length() > 0.0 {
                    angle_diff(heading(), a_total.angle()).abs() < 5.0f64.to_radians()
                } else {
                    false
                };

                if !fuel_economy && target_accel_component > 100.0 && aimed_correctly {
                    activate_ability(Ability::Boost);
                } else {
                    deactivate_ability(Ability::Boost);
                }

                // Draw projected intercept point of the currently selected target
                if v_c > 0.0 {
                    draw_diamond(intercept_point, 16.0, rgb(255, 0, 0));
                    draw_line(position(), intercept_point, rgb(255, 0, 0));
                    draw_line(target_pos, intercept_point, rgb(0, 255, 0)); // Draw vector from target to intercept in green
                    draw_text!(intercept_point + vec2(0.0, 20.0), rgb(255, 0, 0), "Intercept: {:.2}s", time_to_intercept);
                }

                // Draw the current position of the currently selected target
                draw_square(target_pos, 20.0, rgb(255, 0, 0));
            }
        } else {
            // No target - burn straight ahead at maximum speed until we find a lock, provided we retain fuel
            let mode = self.determine_guidance_mode(false, false, false);
            let a_cmd = if mode == "Search Mode" {
                let heading_dir = vec2(heading().cos(), heading().sin());
                heading_dir * max_forward_acceleration()
            } else {
                vec2(0.0, 0.0)
            };
            debug!("Mode: {}", mode);
            debug!("Acc X: {:.2}", a_cmd.x);
            debug!("Acc Y: {:.2}", a_cmd.y);

            accelerate(a_cmd);
            deactivate_ability(Ability::Boost);
        }
    }
}

/// Computes the optimal constant-acceleration vector to intercept a target,
/// given our position and velocity, and the target's position, velocity, and acceleration.
pub fn optimal_intercept_acceleration(
    our_pos: Vec2,
    our_vel: Vec2,
    target_pos: Vec2,
    target_vel: Vec2,
    target_accel: Vec2,
    max_accel: f64,
) -> Option<Vec2> {
    let x = target_pos - our_pos;
    let v_rel = target_vel - our_vel;

    // We want to solve for time-to-impact T > 0 in:
    // |2x/T^2 + 2v_rel/T + target_accel|^2 = max_accel^2
    // Which is f(T) = |P(T)|^2 - max_accel^2 * T^4 = 0
    // where P(T) = 2x + 2v_rel*T + target_accel*T^2
    let dist = x.length();
    let rel_speed = v_rel.length();
    let mut t = if rel_speed > 1.0 {
        dist / rel_speed
    } else {
        (2.0 * dist / max_accel).sqrt()
    };
    t = t.max(0.01);

    let f = |t: f64| {
        let p = 2.0 * x + 2.0 * v_rel * t + target_accel * t * t;
        p.dot(p) - max_accel.powi(2) * t.powi(4)
    };

    let df = |t: f64| {
        let p = 2.0 * x + 2.0 * v_rel * t + target_accel * t * t;
        let p_prime = 2.0 * v_rel + 2.0 * target_accel * t;
        2.0 * p.dot(p_prime) - 4.0 * max_accel.powi(2) * t.powi(3)
    };

    let clamp = |t: f64| t.max(0.001);

    if let Some(time_to_impact) = newton_solve(t, f, df, clamp, 30, 1e-4) {
        if time_to_impact > 0.0 {
            let u = (2.0 * x) / (time_to_impact * time_to_impact) + (2.0 * v_rel) / time_to_impact + target_accel;
            if u.length() > 0.0 {
                return Some(u.normalize() * max_accel);
            }
        }
    }
    None
}

pub struct MissileAimer {
    pub cruise_speed: f64,
}

impl MissileAimer {
    pub fn new(cruise_speed: f64) -> Self {
        Self { cruise_speed }
    }

    pub fn calculate_intercept(
        target: &KinematicState,
        missile_pos: Vec2,
        missile_vel: Vec2,
        current_tick: u32,
    ) -> (f64, Vec2) {
        let target_pos = target.position_at(current_tick);
        let target_vel = target.velocity_at(current_tick);
        let r = target_pos - missile_pos;
        let r_len = r.length();
        if r_len < 1e-6 {
            return (0.0, target_pos);
        }
        let dir = r / r_len;
        let v_rel = target_vel - missile_vel;
        let v_c = -v_rel.dot(dir);
        let t_go = if v_c > 0.0 { r_len / v_c } else { f64::MAX };

        let vel_weight = ((v_c - 100.0) / 300.0).clamp(0.0, 1.0);
        let accel_weight = ((5.0 - t_go) / 2.0).clamp(0.0, 1.0);

        let intercept_tick = current_tick + (t_go / TICK_LENGTH).round() as u32;
        let dt = (intercept_tick as f64 - target.last_scanned as f64) * TICK_LENGTH;
        let intercept_point = target.position + (target.velocity * vel_weight) * dt + 0.5 * (target.acceleration * accel_weight) * dt * (dt + TICK_LENGTH);

        (t_go, intercept_point)
    }
}

impl AimAt for MissileAimer {
    fn aim_at(
        &self,
        target: &KinematicState,
        us: &KinematicState,
    ) -> Option<(Vec2, f64)> {
        let target_pos = target.position_at(current_tick());
        let r = target_pos - us.position;
        let r_len = r.length();
        let dir = if r_len > 1e-6 { r / r_len } else { Vec2::new(us.heading.unwrap_or(0.0).cos(), us.heading.unwrap_or(0.0).sin()) };
        let estimated_missile_vel = us.velocity + dir * self.cruise_speed;

        let (_, intercept_point) = Self::calculate_intercept(target, us.position, estimated_missile_vel, current_tick());
        let d = intercept_point - us.position;
        let d_len = d.length();
        if d_len > 1e-6 {
            let aim_dir = d.normalize();

            // Extrapolate the aim point next tick
            let target_pos_next = target.position_at(current_tick() + 1);
            let us_pos_next = us.position + us.velocity * TICK_LENGTH;
            let r_next = target_pos_next - us_pos_next;
            let r_len_next = r_next.length();
            let dir_next = if r_len_next > 1e-6 { r_next / r_len_next } else { dir };
            let estimated_missile_vel_next = us.velocity + dir_next * self.cruise_speed;

            let (_, intercept_point_next) = Self::calculate_intercept(target, us_pos_next, estimated_missile_vel_next, current_tick() + 1);
            let d_next = intercept_point_next - us_pos_next;
            
            let omega = if d_next.length() > 0.0 {
                angle_diff(aim_dir.angle(), d_next.angle()) / TICK_LENGTH
            } else {
                0.0
            };

            Some((aim_dir, omega))
        } else {
            None
        }
    }
}

#[cfg(test)]
mod missile_test;
