use oort_api::prelude::*;
use crate::radar::{RadarController, Contact, RADIO_PING_SNR};
use crate::control::{quick_turn, quick_turn_with_target_omega, AngleTracker};
use crate::physics::KinematicState;
use crate::aim::AimAt;

// Linearly extrapolates target position assuming zero target acceleration.
// Target acceleration estimates can be volatile and introduce noise/fluctuations
// into long-term intercept prediction.
fn target_position_at(target: &KinematicState, tick: u32) -> Vec2 {
    let dt = (tick - target.last_scanned) as f64 * TICK_LENGTH;
    target.position + target.velocity * dt
}

// Linearly extrapolates target velocity assuming zero target acceleration.
fn target_velocity_at(target: &KinematicState, _tick: u32) -> Vec2 {
    target.velocity
}

const ALIGNMENT_TIME_THRESHOLD_SEC: f64 = 5.0;
const ALIGNMENT_ERROR_LARGE_DEG: f64 = 3.0;
const ALIGNMENT_ERROR_SMALL_DEG: f64 = 0.25;

/// Telemetry data for a tracked target, transmitted securely over radio.
#[derive(Clone, Copy, Debug)]
pub struct TargetTelemetry {
    pub position: Vec2,
    pub velocity: Vec2,
    pub rssi: f32,
    pub class: Class,
    pub tick: u8,
}

#[derive(Clone, Copy, Debug)]
pub struct LoiterCommand {
    pub aim_point: Vec2,
    pub cruise_speed: f64,
}

#[derive(Clone, Copy, Debug)]
pub enum MissileMessage {
    Telemetry(TargetTelemetry),
    Loiter(LoiterCommand),
}

impl MissileMessage {
    pub fn serialize(&self) -> [u8; 30] {
        let mut payload = [0u8; 30];
        match self {
            MissileMessage::Telemetry(t) => {
                payload[0] = 0;
                payload[1..5].copy_from_slice(&(t.position.x as f32).to_le_bytes());
                payload[5..9].copy_from_slice(&(t.position.y as f32).to_le_bytes());
                payload[9..13].copy_from_slice(&(t.velocity.x as f32).to_le_bytes());
                payload[13..17].copy_from_slice(&(t.velocity.y as f32).to_le_bytes());
                payload[17..21].copy_from_slice(&t.rssi.to_le_bytes());
                payload[21] = t.tick;
                payload[22] = match t.class {
                    Class::Fighter => 1,
                    Class::Frigate => 2,
                    Class::Cruiser => 3,
                    Class::Missile => 4,
                    Class::Torpedo => 5,
                    Class::Target => 6,
                    _ => 0,
                };
            }
            MissileMessage::Loiter(l) => {
                payload[0] = 1;
                payload[1..5].copy_from_slice(&(l.aim_point.x as f32).to_le_bytes());
                payload[5..9].copy_from_slice(&(l.aim_point.y as f32).to_le_bytes());
                payload[9..13].copy_from_slice(&(l.cruise_speed as f32).to_le_bytes());
            }
        }
        payload
    }

    pub fn deserialize(payload: &[u8; 30]) -> Self {
        match payload[0] {
            1 => {
                let aim_x = f32::from_le_bytes(payload[1..5].try_into().unwrap()) as f64;
                let aim_y = f32::from_le_bytes(payload[5..9].try_into().unwrap()) as f64;
                let speed = f32::from_le_bytes(payload[9..13].try_into().unwrap()) as f64;
                MissileMessage::Loiter(LoiterCommand {
                    aim_point: vec2(aim_x, aim_y),
                    cruise_speed: speed,
                })
            }
            _ => {
                let pos_x = f32::from_le_bytes(payload[1..5].try_into().unwrap()) as f64;
                let pos_y = f32::from_le_bytes(payload[5..9].try_into().unwrap()) as f64;
                let vel_x = f32::from_le_bytes(payload[9..13].try_into().unwrap()) as f64;
                let vel_y = f32::from_le_bytes(payload[13..17].try_into().unwrap()) as f64;
                let rssi = f32::from_le_bytes(payload[17..21].try_into().unwrap());
                let tick = payload[21];
                let class = match payload[22] {
                    1 => Class::Fighter,
                    2 => Class::Frigate,
                    3 => Class::Cruiser,
                    4 => Class::Missile,
                    5 => Class::Torpedo,
                    6 => Class::Target,
                    _ => Class::Fighter,
                };
                MissileMessage::Telemetry(TargetTelemetry {
                    position: vec2(pos_x, pos_y),
                    velocity: vec2(vel_x, vel_y),
                    rssi,
                    class,
                    tick,
                })
            }
        }
    }
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
                let msg = MissileMessage::Telemetry(telemetry);
                self.missile_radio.transmit(msg.serialize());
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
    pub cruise_speed: f64,
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
    pub aim_point: Option<Vec2>,
    pub is_cruising: bool,
    pub cruise_aim_point: Option<Vec2>,
    pub cruise_target_speed: Option<f64>,
    pub received_first_message: bool,
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
            cruise_speed: 800.0,
            min_search_fuel: 500.0,
            turn_safety_buffer_ticks: 1.0,

            radar_controller: RadarController::new(),
            angle_tracker: AngleTracker::new(5.0),
            initial_fuel: fuel(),
            target_id: None,
            first_detection_tick: None,
            target_channel: 3,
            secure_radio: None,
            aim_point: None,
            is_cruising: false,
            cruise_aim_point: None,
            cruise_target_speed: None,
            received_first_message: false,
        }
    }

    fn estimate_t_go(&self, target: &Contact) -> f64 {
        let us = KinematicState::self_state();
        let r = target.current_position() - position();
        let r_len = r.length();
        if r_len < 1e-6 {
            return 0.0;
        }
        let v_rel = target.current_velocity() - velocity();
        let v_c = -v_rel.dot(r) / r_len;

        let fuel_economy = self.check_fuel_economy(v_c, r_len, target.class);
        let fwd_budget = if fuel_economy { 0.0 } else { max_forward_acceleration() };

        let t_go = if let Some(mei) = minimum_effort_intercept(&us, &target.kinematic, fwd_budget) {
            mei.constant_velocity.t_go
        } else {
            let v_cruise = self.cruise_target_speed.unwrap_or(self.cruise_speed);
            let v_c_clamped = v_c.max(if self.is_cruising { v_cruise } else { velocity().length() }).max(0.1);
            r_len / v_c_clamped
        };

        debug!("t_go: {} (extrapolated)", t_go.round() as i32);
        debug!("  r_len: {}", r_len.round() as i32);
        debug!("  v_c: {}", v_c.round() as i32);
        debug!("  self.is_cruising: {}", self.is_cruising);

        t_go
    }

    fn calculate_aim_point(&self, target: &Contact, t_go: f64) -> Vec2 {
        let current_t = current_tick();
        let intercept_tick = current_t + (t_go / TICK_LENGTH).round() as u32;
        target_position_at(&target.kinematic, intercept_tick)
    }

    fn calculate_thrust(
        &self,
        target: &Contact,
        aim_point: Vec2,
        t_go: f64,
        v_c: f64,
        los_rate: f64,
        e_perp: Vec2,
        fuel_economy: bool,
        in_nez: bool,
    ) -> Vec2 {
        let p = position();
        let v = velocity();
        
        let r_aim = aim_point - p;
        let r_aim_len = r_aim.length();
        let dir_aim = if r_aim_len > 1e-6 { r_aim / r_aim_len } else { vec2(heading().cos(), heading().sin()) };
        let perp_aim = vec2(-dir_aim.y, dir_aim.x);
        
        let travel_angle = v.angle();
        let aim_angle = dir_aim.angle();
        let angular_error = angle_diff(travel_angle, aim_angle).abs();
        
        let allowed_error = if t_go > ALIGNMENT_TIME_THRESHOLD_SEC {
            ALIGNMENT_ERROR_LARGE_DEG.to_radians()
        } else {
            ALIGNMENT_ERROR_SMALL_DEG.to_radians()
        };
        
        let a_max = max_forward_acceleration();
        let mut a_total = vec2(0.0, 0.0);
        let mut aligned = true;
        let mut cruise_speed_reached = false;
        
        // 1. Align our travel direction with our aim point
        if angular_error > allowed_error {
            aligned = false;
            let v_perp = v.dot(perp_aim);
            let a_perp_req = -v_perp / TICK_LENGTH;
            let a_perp_clamped = a_perp_req.clamp(-a_max, a_max);
            a_total = a_perp_clamped * perp_aim;
        }
        
        let a_total_len = a_total.length();
        let spare_acc = (a_max * a_max - a_total_len * a_total_len).max(0.0).sqrt();
        
        // 2. Reach our cruise velocity if aligned or spare acceleration is available
        if aligned || spare_acc > 1e-6 {
            let v_parallel = v.dot(dir_aim);
            let fuel_spent = self.initial_fuel - fuel();
            if v_parallel >= self.cruise_speed && fuel_spent >= self.cruise_speed {
                cruise_speed_reached = true;
            } else {
                let a_parallel_req = if fuel_spent < self.cruise_speed {
                    spare_acc
                } else {
                    (self.cruise_speed - v_parallel) / TICK_LENGTH
                };
                let a_parallel_clamped = a_parallel_req.clamp(-spare_acc, spare_acc);
                a_total += a_parallel_clamped * dir_aim;
            }
        }
        
        // 3. Fallback to existing calculations if both aligned and cruise speed reached
        if aligned && cruise_speed_reached && in_nez {
            if fuel_economy {
                let a_lateral = self.pn_gain * v_c.max(self.pn_min_vc) * los_rate * e_perp;
                let r_target = target_position_at(&target.kinematic, current_tick()) - p;
                let r_target_len = r_target.length();
                let dir = if r_target_len > 1e-6 {
                    r_target / r_target_len
                } else {
                    vec2(heading().cos(), heading().sin())
                };
                let forward_acc = 0.0;
                a_total = a_lateral + dir * forward_acc;
            } else {
                let r_intercept = aim_point - p;
                let dir_intercept = if r_intercept.length() > 1e-6 {
                    r_intercept.normalize()
                } else {
                    vec2(heading().cos(), heading().sin())
                };

                let perp_dir = vec2(-dir_intercept.y, dir_intercept.x);
                let v_perp = v.dot(perp_dir);
                let alpha = if v_perp.abs() <= 100.0 {
                    (-v_perp / 100.0).asin()
                } else {
                    -v_perp.signum() * std::f64::consts::FRAC_PI_2
                };
                let desired_boost_heading = dir_intercept.angle() + alpha;
                let acc_dir = vec2(desired_boost_heading.cos(), desired_boost_heading.sin());
                a_total = acc_dir * (max_forward_acceleration() + 100.0);
            }
        }
        
        a_total
    }

    fn calculate_nez_metric(&self, target: &Contact, aim_point: Vec2, t_go: f64) -> f64 {
        let p = position();
        let v = velocity();
        
        let enemy_stats = target.class.default_stats();
        let a_enemy_base = enemy_stats.max_forward_acceleration
            .max(enemy_stats.max_backward_acceleration)
            .max(enemy_stats.max_lateral_acceleration);
        
        let num_intervals = (t_go / 10.0).ceil();
        let enemy_boost_dv = if target.class == Class::Fighter || target.class == Class::Missile {
            num_intervals * 100.0
        } else {
            0.0
        };
        
        let enemy_max_acceleration_sum = a_enemy_base * t_go + enemy_boost_dv;
        let enemy_displacement = 0.5 * enemy_max_acceleration_sum * t_go;
        
        let p_intercept_missile = p + v * t_go;
        let intercept_tick = current_tick() + (t_go / TICK_LENGTH).round() as u32;
        let p_intercept_target = target_position_at(&target.kinematic, intercept_tick);
        let error_vector = p_intercept_target - p_intercept_missile;
        
        let r_aim = aim_point - p;
        let dir_aim = if r_aim.length() > 1e-6 { r_aim.normalize() } else { vec2(heading().cos(), heading().sin()) };
        let perp_aim = vec2(-dir_aim.y, dir_aim.x);
        let lateral_error = error_vector.dot(perp_aim).abs();
        
        let a_max = max_forward_acceleration();
        let our_max_acceleration_sum = (a_max * t_go).min(fuel());
        let our_displacement = 0.5 * our_max_acceleration_sum * t_go;
        
        const SPARE_DV_REQUIRED: f64 = 150.0;
        our_displacement - (enemy_displacement + lateral_error) - SPARE_DV_REQUIRED
    }

    fn determine_guidance_mode(
        &self,
        has_target: bool,
        is_terminal: bool,
        fuel_economy: bool,
    ) -> &'static str {
        if self.is_cruising {
            "Cruise Mode"
        } else if !has_target {
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
        v_c >= self.cruise_speed && fuel() < possible_enemy_dv
    }

    pub fn tick(&mut self) {
        let prev_target_id = self.target_id;
        if let Some(ref sr) = self.secure_radio {
            // Secure radio mode
            if let Some(payload) = sr.receive() {
                let msg = MissileMessage::deserialize(&payload);
                debug!("Missile received secure radio message: {:?}", msg);
                if !self.received_first_message {
                    self.received_first_message = true;
                    match msg {
                        MissileMessage::Loiter(cmd) => {
                            self.is_cruising = true;
                            self.cruise_aim_point = Some(cmd.aim_point);
                            self.cruise_target_speed = Some(cmd.cruise_speed);
                            debug!("Missile entering cruise mode. Aim point: {:?}, Speed: {}", cmd.aim_point, cmd.cruise_speed);
                        }
                        MissileMessage::Telemetry(telemetry) => {
                            debug!("Decoded secure radio ping: pos=({:.1}, {:.1}) vel=({:.1}, {:.1}) rssi={} class={:?}", telemetry.position.x, telemetry.position.y, telemetry.velocity.x, telemetry.velocity.y, telemetry.rssi, telemetry.class);
                            let target_id = self.radar_controller.add_radio_ping(telemetry);
                            if self.target_id.is_none() {
                                self.target_id = Some(target_id);
                            }
                        }
                    }
                } else {
                    match msg {
                        MissileMessage::Telemetry(telemetry) => {
                            debug!("Decoded secure radio ping (subsequent): pos=({:.1}, {:.1}) vel=({:.1}, {:.1}) rssi={} class={:?}", telemetry.position.x, telemetry.position.y, telemetry.velocity.x, telemetry.velocity.y, telemetry.rssi, telemetry.class);
                            self.radar_controller.add_radio_ping(telemetry);
                            if !self.is_cruising && self.target_id.is_none() {
                                self.target_id = Some(self.radar_controller.add_radio_ping(telemetry));
                            }
                        }
                        MissileMessage::Loiter(_cmd) => {
                            // Ignore subsequent loiter commands intended for other missiles
                        }
                    }
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
                    self.received_first_message = true;
                    self.radar_controller.add_radio_ping(telemetry);
                }
            }
        }

        // Set current target as high priority in the radar controller
        self.radar_controller.priority_targets = self.target_id.map(|id| vec![id]).unwrap_or_default();
        self.radar_controller.update();
        let contacts = self.radar_controller.contacts();
        let current_t = current_tick();

        // If we are cruising, check if any contact has entered our NEZ or if t_go is below 10s
        if self.is_cruising {
            let mut found_target = None;
            for c in contacts.iter() {
                if c.class != Class::Missile && c.class != Class::Torpedo {
                    let (t_go, aim_point) = MissileAimer::calculate_intercept(&c.kinematic, position(), velocity(), 0.0, current_tick());
                    let nez_metric = self.calculate_nez_metric(c, aim_point, t_go);
                    if nez_metric >= 0.0 || t_go < 10.0 {
                        found_target = Some(c.id);
                        debug!("Encountered enemy ID {} (nez_metric: {}, t_go: {:.2}s). Locking on and exiting cruise mode.", c.id, nez_metric, t_go);
                        break;
                    }
                }
            }
            if let Some(target_id) = found_target {
                self.is_cruising = false;
                self.target_id = Some(target_id);
            }
        }

        if !self.is_cruising {
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
        }

        if self.target_id != prev_target_id {
            self.aim_point = None;
        }

        if let Some(tid) = self.target_id {
            if let Some(target) = contacts.iter().find(|c| c.id == tid) {
                let target_pos = target_position_at(&target.kinematic, current_tick());
                let target_vel = target_velocity_at(&target.kinematic, current_tick());
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
                let time_to_intercept = self.estimate_t_go(target);
                let aim_point = self.calculate_aim_point(target, time_to_intercept);
                self.aim_point = Some(aim_point);
                debug!("Time to intercept: {:}s", time_to_intercept);

                let nez_metric = self.calculate_nez_metric(target, aim_point, time_to_intercept);
                debug!("NEZ Metric: {}", nez_metric.round() as i32);
                debug!("Target in NEZ: {}", nez_metric >= 0.0);

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

                let in_nez = nez_metric >= 0.0;
                let a_total = self.calculate_thrust(
                    target,
                    aim_point,
                    time_to_intercept,
                    v_c,
                    los_rate,
                    e_perp,
                    fuel_economy,
                    in_nez,
                );

                // Turn to point directly at target intercept point when it's time to explode
                let time_until_explosion = (time_to_intercept - self.proximity_ticks * TICK_LENGTH).max(0.0);

                // Heading we need to face at the moment of explosion
                let position_at_explosion = position() + time_until_explosion * velocity();
                let target_pos_at_explosion = target_position_at(&target.kinematic, current_tick() + (time_until_explosion / TICK_LENGTH).round() as u32);
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

                if time_until_explosion <= turn_time_with_buffer {
                    let r = target_pos_at_explosion - position();
                    let target_angle = r.angle();
                    let r_len_sq = r.dot(r);
                    let target_omega = if r_len_sq > 1e-6 {
                        (-r.x * velocity().y + r.y * velocity().x) / r_len_sq
                    } else {
                        0.0
                    };
                    quick_turn_with_target_omega(target_angle, target_omega);
                } else {
                    quick_turn(a_total.angle());
                }

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
                    draw_diamond(aim_point, 16.0, rgb(255, 0, 0));
                    draw_line(position(), aim_point, rgb(255, 0, 0));
                    draw_line(target_pos, aim_point, rgb(0, 255, 0)); // Draw vector from target to intercept in green
                    draw_text!(aim_point + vec2(0.0, 20.0), rgb(255, 0, 0), "Intercept: {:.2}s", time_to_intercept);
                }

                // Draw the current position of the currently selected target
                draw_square(target_pos, 20.0, rgb(255, 0, 0));
            } else {
                self.aim_point = None;
            }
        } else {
            self.aim_point = None;
            if self.is_cruising {
                let cruise_speed = self.cruise_target_speed.unwrap_or(self.cruise_speed);
                let aim_point = self.cruise_aim_point.unwrap_or(position());

                let r_aim = aim_point - position();
                let r_aim_len = r_aim.length();
                let dir_aim = if r_aim_len > 1e-6 { r_aim / r_aim_len } else { vec2(heading().cos(), heading().sin()) };
                let perp_aim = vec2(-dir_aim.y, dir_aim.x);
                let v = velocity();
                let travel_angle = v.angle();
                let aim_angle = dir_aim.angle();
                let angular_error = angle_diff(travel_angle, aim_angle).abs();

                let allowed_error = ALIGNMENT_ERROR_LARGE_DEG.to_radians();
                let a_max = max_forward_acceleration();
                let mut a_total = vec2(0.0, 0.0);
                let mut aligned = true;

                if angular_error > allowed_error {
                    aligned = false;
                    let v_perp = v.dot(perp_aim);
                    let a_perp_req = -v_perp / TICK_LENGTH;
                    let a_perp_clamped = a_perp_req.clamp(-a_max, a_max);
                    a_total = a_perp_clamped * perp_aim;
                }

                let a_total_len = a_total.length();
                let spare_acc = (a_max * a_max - a_total_len * a_total_len).max(0.0).sqrt();

                if aligned || spare_acc > 1e-6 {
                    let v_parallel = v.dot(dir_aim);
                    let a_parallel_req = (cruise_speed - v_parallel) / TICK_LENGTH;
                    let a_parallel_clamped = a_parallel_req.clamp(-spare_acc, spare_acc);
                    a_total += a_parallel_clamped * dir_aim;
                }

                quick_turn(a_total.angle());
                accelerate(a_total);
                deactivate_ability(Ability::Boost);

                let mode = self.determine_guidance_mode(false, false, false);
                debug!("Mode: {}", mode);
                debug!("Acc X: {:.2}", a_total.x);
                debug!("Acc Y: {:.2}", a_total.y);
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
        debug!("Missile current aim_point: {:?}", self.aim_point.or_else(|| self.cruise_aim_point));
    }
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
        cruise_speed: f64,
        current_tick: u32,
    ) -> (f64, Vec2) {
        let missile = KinematicState::new(
            Class::Missile,
            missile_pos,
            missile_vel,
            Vec2::new(0.0, 0.0),
            current_tick,
        );

        let target_pos = target_position_at(target, current_tick);
        let r = target_pos - missile_pos;
        let r_len = r.length();
        if r_len < 1e-6 {
            return (0.0, target_pos);
        }
        let target_vel = target_velocity_at(target, current_tick);
        let v_rel = target_vel - missile_vel;
        let v_c = -v_rel.dot(r) / r_len;

        if let Some(mei) = minimum_effort_intercept(&missile, target, 0.0) {
            (mei.constant_velocity.t_go, mei.constant_velocity.position)
        } else {
            let v_c_clamped = v_c.max(cruise_speed).max(0.1);
            let t_go = r_len / v_c_clamped;
            let intercept_tick = current_tick + (t_go / TICK_LENGTH).round() as u32;
            let intercept_point = target_position_at(target, intercept_tick);
            (t_go, intercept_point)
        }
    }
}

impl AimAt for MissileAimer {
    fn aim_at(
        &self,
        target: &KinematicState,
        us: &KinematicState,
    ) -> Option<(Vec2, f64)> {
        let target_pos = target_position_at(target, current_tick());
        let r = target_pos - us.position;
        let r_len = r.length();
        let dir = if r_len > 1e-6 { r / r_len } else { Vec2::new(us.heading.unwrap_or(0.0).cos(), us.heading.unwrap_or(0.0).sin()) };
        let estimated_missile_vel = us.velocity + dir * self.cruise_speed;

        let (_, intercept_point) = Self::calculate_intercept(target, us.position, estimated_missile_vel, self.cruise_speed, current_tick());
        let d = intercept_point - us.position;
        let d_len = d.length();
        if d_len > 1e-6 {
            let aim_dir = d.normalize();

            // Extrapolate the aim point next tick
            let target_pos_next = target_position_at(target, current_tick() + 1);
            let us_pos_next = us.position + us.velocity * TICK_LENGTH;
            let r_next = target_pos_next - us_pos_next;
            let r_len_next = r_next.length();
            let dir_next = if r_len_next > 1e-6 { r_next / r_len_next } else { dir };
            let estimated_missile_vel_next = us.velocity + dir_next * self.cruise_speed;

            let (_, intercept_point_next) = Self::calculate_intercept(target, us_pos_next, estimated_missile_vel_next, self.cruise_speed, current_tick() + 1);
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

#[derive(Clone, Debug)]
pub struct InterceptResult {
    pub position: Vec2,
    pub t_go: f64,
    pub fuel_consumed: f64,
}

#[derive(Clone, Debug)]
pub struct MinimumEffortIntercept {
    pub constant_velocity: InterceptResult,
    pub worst_case_positive: InterceptResult,
    pub worst_case_negative: InterceptResult,
}

pub fn minimum_effort_intercept(
    missile: &KinematicState,
    enemy: &KinematicState,
    forward_accel_budget: f64,
) -> Option<MinimumEffortIntercept> {
    let r0_vec = enemy.position - missile.position;
    let r0 = r0_vec.length();
    if r0 < 1e-6 {
        let res = InterceptResult {
            position: enemy.position,
            t_go: 0.0,
            fuel_consumed: 0.0,
        };
        return Some(MinimumEffortIntercept {
            constant_velocity: res.clone(),
            worst_case_positive: res.clone(),
            worst_case_negative: res.clone(),
        });
    }

    let u0 = r0_vec / r0;
    let w0 = Vec2::new(-u0.y, u0.x);
    let v_rel = enemy.velocity - missile.velocity;
    let v_c = -v_rel.dot(u0);

    if v_c <= 0.0 {
        return None;
    }

    let max_fwd_acc = if cfg!(test) { 250.0 } else { max_forward_acceleration() };
    let max_lat_acc = if cfg!(test) { 300.0 } else { max_forward_acceleration() };
    let available_fuel = if cfg!(test) { 10000.0 } else { fuel() };
    let a_fwd = forward_accel_budget.min(max_fwd_acc);

    let t_go = if a_fwd > 1e-6 {
        (-v_c + (v_c * v_c + 2.0 * a_fwd * r0).sqrt()) / a_fwd
    } else {
        r0 / v_c
    };

    let enemy_max = crate::physics::max_acceleration_over_time(enemy.class, t_go);
    let v_rel_perp = v_rel.dot(w0);
    let c_val = 2.0 * v_rel_perp / t_go;

    let l_val = if available_fuel / t_go >= a_fwd {
        ((available_fuel / t_go).powi(2) - a_fwd * a_fwd).sqrt()
    } else {
        0.0
    };
    let m_val = max_lat_acc.min(l_val);

    let a_e_reach_min = -c_val - m_val;
    let a_e_reach_max = -c_val + m_val;

    // Draw full enemy line at T in red
    let p_enemy_min = enemy.position + enemy.velocity * t_go - 0.5 * enemy_max * t_go * t_go * w0;
    let p_enemy_max = enemy.position + enemy.velocity * t_go + 0.5 * enemy_max * t_go * t_go * w0;
    draw_line(p_enemy_min, p_enemy_max, rgb(255, 0, 0));

    // Draw perpendicular green tick marks at either end of the reachable segment
    let red_len = (p_enemy_max - p_enemy_min).length();
    let tick_len = red_len / 5.0;

    let p_reach_min = enemy.position + enemy.velocity * t_go + 0.5 * a_e_reach_min * t_go * t_go * w0;
    let p_reach_max = enemy.position + enemy.velocity * t_go + 0.5 * a_e_reach_max * t_go * t_go * w0;

    draw_line(p_reach_min - u0 * (tick_len / 2.0), p_reach_min + u0 * (tick_len / 2.0), rgb(0, 255, 0));
    draw_line(p_reach_max - u0 * (tick_len / 2.0), p_reach_max + u0 * (tick_len / 2.0), rgb(0, 255, 0));


    let evaluate_intercept = |a_e_val: f64| -> InterceptResult {
        let target_pos = enemy.position + enemy.velocity * t_go + 0.5 * a_e_val * t_go * t_go * w0;
        let a_m_lat = c_val + a_e_val;
        let a_total = (a_fwd * a_fwd + a_m_lat * a_m_lat).sqrt();
        let fuel_consumed = a_total * t_go;
        InterceptResult {
            position: target_pos,
            t_go,
            fuel_consumed,
        }
    };

    let constant_velocity = evaluate_intercept(0.0);
    let worst_case_positive = evaluate_intercept(enemy_max);
    let worst_case_negative = evaluate_intercept(-enemy_max);

    let success = m_val >= c_val.abs() + enemy_max;
    if !success {
        return None;
    }

    Some(MinimumEffortIntercept {
        constant_velocity,
        worst_case_positive,
        worst_case_negative,
    })
}

#[cfg(test)]
mod missile_test;
