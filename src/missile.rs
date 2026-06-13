use crate::aim::AimAt;
use crate::control::{AngleTracker, quick_turn, quick_turn_with_target_omega};
use crate::physics::KinematicState;
use crate::radar::{Contact, RadarController};
use oort_api::prelude::*;

// Flak-dodging constants:
// N: extra delta-v (m/s) threshold. Also, offset distance is N/2 meters.
const N: f64 = 150.0;
// M: time remaining until intercept (seconds) below which we snap back to the target.
const M: f64 = 1.5;

pub const DEFAULT_MIN_SEARCH_FUEL: f64 = 500.0;

// Linearly extrapolates target position assuming zero target acceleration.
// Target acceleration estimates can be volatile and introduce noise/fluctuations
// into long-term intercept prediction.
fn target_position_at(target: &KinematicState, tick: u32) -> Vec2 {
    let dt = (tick - target.last_scanned) as f64 * TICK_LENGTH;
    target.position + target.velocity * dt
}

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
    pub min_search_fuel: f64,
    pub turn_safety_buffer_ticks: f64,

    // State
    pub radar_controller: RadarController,
    pub angle_tracker: AngleTracker,
    pub initial_fuel: f64,
    pub target_id: Option<u32>,
    pub target_channel: usize,
    pub secure_radio: Option<crate::radio::SecureRadio>,
    pub aim_point: Option<Vec2>,
    pub cruise_aim_point: Option<Vec2>,
    pub has_entered_nez: bool,
    pub dodge_sign: f64,
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

        let mut radar_controller = RadarController::new();
        radar_controller.jamming_mode = true;

        Self {
            proximity_dist: 20.0,
            proximity_ticks: (0.1 / TICK_LENGTH) - 1.0, // Missile bullets last for 200 ms.
            pn_gain: 4.0,
            pn_min_vc: 100.0,
            min_search_fuel: DEFAULT_MIN_SEARCH_FUEL,
            turn_safety_buffer_ticks: 1.0,

            radar_controller,
            angle_tracker: AngleTracker::new(5.0),
            initial_fuel: fuel(),
            target_id: None,
            target_channel: 3,
            secure_radio: None,
            aim_point: None,
            cruise_aim_point: None,
            has_entered_nez: false,
            dodge_sign: if cfg!(test) {
                1.0
            } else if rand(0.0, 1.0) < 0.5 {
                -1.0
            } else {
                1.0
            },
        }
    }

    pub fn tick(&mut self) {
        let prev_target_id = self.target_id;

        // 1. Parse radio message
        if let Some(ref sr) = self.secure_radio {
            if let Some(payload) = sr.receive() {
                let telemetry = TargetTelemetry::deserialize(&payload);
                self.radar_controller.add_radio_ping(telemetry);
            }
            sr.prepare_receive();
        } else {
            select_radio(0);
            set_radio_channel(self.target_channel);
            if let Some(msg) = receive() {
                let pos_x = msg[0];
                let pos_y = msg[1];
                let vel_x = msg[2];
                let vel_y = msg[3];

                if pos_x.is_finite()
                    && pos_y.is_finite()
                    && vel_x.is_finite()
                    && vel_y.is_finite()
                    && pos_x.abs() < 100_000.0
                    && pos_y.abs() < 100_000.0
                    && vel_x.abs() < 10_000.0
                    && vel_y.abs() < 10_000.0
                {
                    let telemetry = TargetTelemetry {
                        position: vec2(pos_x, pos_y),
                        velocity: vec2(vel_x, vel_y),
                        rssi: 0.0,
                        class: Class::Fighter,
                        tick: current_tick() as u8,
                    };
                    self.radar_controller.add_radio_ping(telemetry);
                }
            }
        }

        // 2. Update radar
        self.radar_controller.priority_targets =
            self.target_id.map(|id| vec![id]).unwrap_or_default();
        self.radar_controller.update();
        let contacts = self.radar_controller.contacts();

        // Target selection
        let target_still_valid = if let Some(id) = self.target_id {
            contacts
                .iter()
                .any(|c| c.id == id && c.class != Class::Missile)
        } else {
            false
        };

        if !target_still_valid {
            self.target_id = contacts
                .iter()
                .find(|c| c.class != Class::Missile)
                .map(|c| c.id);
        }

        if self.target_id != prev_target_id {
            self.aim_point = None;
            self.has_entered_nez = false;
        }

        // Construct snapshots
        let mut target_snapshot = None;
        if let Some(tid) = self.target_id {
            if let Some(contact) = contacts.iter().find(|c| c.id == tid) {
                target_snapshot = Some(TargetSnapshot {
                    position: contact.current_position(),
                    velocity: contact.current_velocity(),
                    class: contact.class,
                    last_scanned: contact.kinematic.last_scanned,
                });
            }
        }

        let snapshot = StateSnapshot {
            position: position(),
            velocity: velocity(),
            heading: heading(),
            angular_velocity: angular_velocity(),
            fuel: fuel(),
            current_tick: current_tick(),
            max_forward_acceleration: if cfg!(test) {
                250.0
            } else {
                max_forward_acceleration()
            },
            max_lateral_acceleration: if cfg!(test) {
                300.0
            } else {
                max_lateral_acceleration()
            },
            max_angular_acceleration: if cfg!(test) {
                100.0
            } else {
                max_angular_acceleration()
            },
            target: target_snapshot,
            cruise_point: self.cruise_aim_point,
            has_entered_nez: self.has_entered_nez,
            dodge_sign: self.dodge_sign,
        };

        // Detonation condition
        if check_detonation_condition(&snapshot, self.proximity_ticks) {
            explode();
            return;
        }

        // 3. Determine desired prograde
        let prog_res = determine_prograde(&snapshot, self.pn_gain, self.pn_min_vc);

        // Update state
        self.cruise_aim_point = prog_res.cruise_point;
        self.aim_point = prog_res.aim_point_use;
        if prog_res.nez_metric_passed {
            self.has_entered_nez = true;
        }

        // 4. Determine thrust
        let thrust = determine_thrust(&snapshot, prog_res.prograde, self.min_search_fuel);

        // 5. Determine desired attitude
        let att_res = determine_attitude(
            &snapshot,
            thrust,
            prog_res.prograde,
            self.proximity_ticks,
            self.turn_safety_buffer_ticks,
        );

        // Apply control commands
        if let Some(t) = thrust {
            accelerate(t);
        } else {
            accelerate(vec2(0.0, 0.0));
        }
        if let Some(target_omega) = att_res.target_omega {
            quick_turn_with_target_omega(att_res.desired_heading, target_omega);
        } else {
            quick_turn(att_res.desired_heading);
        }

        let boost_active = if let Some(t) = thrust {
            should_boost(&snapshot, t, self.min_search_fuel)
        } else {
            false
        };
        if boost_active {
            activate_ability(Ability::Boost);
        } else {
            deactivate_ability(Ability::Boost);
        }

        let p = position();
        if let Some(t) = thrust {
            draw_line(p, p + t, rgb(0, 255, 255)); // CYAN for total thrust
        }

        // Draw current prograde and desired prograde as 1km rays
        if snapshot.velocity.length() > 1e-6 {
            let current_prog_vec = snapshot.velocity.normalize();
            draw_line(p, p + current_prog_vec * 1000.0, rgb(255, 0, 255)); // Magenta
        }

        if let Some(desired_prog_vec) = prog_res.prograde {
            let end_point = p + desired_prog_vec * 1000.0;
            draw_line(p, end_point, rgb(0, 255, 0)); // Green
            draw_diamond(end_point, 12.0, rgb(0, 255, 0));
        }

        let desired_heading_dir =
            vec2(att_res.desired_heading.cos(), att_res.desired_heading.sin());
        draw_line(p, p + desired_heading_dir * 150.0, rgb(255, 255, 0)); // Yellow

        // Cruising point is only drawn if the missile is relying on it
        if snapshot.target.is_none() || !prog_res.can_defeat {
            if let Some(cp) = prog_res.cruise_point {
                draw_diamond(cp, 15.0, rgb(0, 128, 255)); // Azure
                draw_line(p, cp, rgb(0, 128, 255));
                draw_text!(cp + vec2(0.0, 20.0), rgb(0, 128, 255), "Cruise Point");
            }
        }

        // Boost active indicator
        let boost_active_debug = if let Some(t) = thrust {
            should_boost(&snapshot, t, self.min_search_fuel)
        } else {
            false
        };
        if boost_active_debug {
            draw_text!(p + vec2(0.0, -60.0), rgb(255, 64, 64), "BOOST ACTIVE");
        }

        // Fuel economy indicator
        if snapshot.fuel < self.min_search_fuel {
            draw_text!(
                p + vec2(0.0, -80.0),
                rgb(255, 128, 0),
                "Fuel Economy: {:.1} < {:.1}",
                snapshot.fuel,
                self.min_search_fuel
            );
        }

        if let Some(target) = snapshot.target {
            let target_pos = target.position_at(snapshot.current_tick);
            let target_vel = target.velocity_at(snapshot.current_tick);
            let v_rel = target_vel - snapshot.velocity;

            // Target relative velocity in ORANGE
            draw_line(target_pos, target_pos + v_rel * 2.0, rgb(255, 128, 0));

            // Target positions
            if let Some(target_use) = prog_res.target_to_use {
                let target_pos_use = target_use.position_at(snapshot.current_tick);
                draw_square(target_pos_use, 20.0, rgb(255, 0, 0)); // Effective target pos in red
                draw_square(target_pos, 10.0, rgb(0, 255, 255)); // Real target pos in cyan
                if prog_res.dodge_active {
                    draw_line(target_pos, target_pos_use, rgb(255, 255, 0)); // Yellow dodge offset line
                }
            }

            // Intercept point visualization
            let r = target_pos - p;
            let v_c = -v_rel.dot(r) / r.length().max(1e-6);
            if v_c > 0.0 {
                if let Some(aim_use) = prog_res.aim_point_use {
                    draw_diamond(aim_use, 16.0, rgb(255, 0, 0));
                    draw_line(p, aim_use, rgb(255, 0, 0));
                    if let Some(target_use) = prog_res.target_to_use {
                        draw_line(
                            target_use.position_at(snapshot.current_tick),
                            aim_use,
                            rgb(0, 255, 0),
                        );
                    }
                }
                if let Some(aim) = prog_res.aim_point {
                    draw_diamond(aim, 10.0, rgb(0, 255, 255));
                    draw_line(p, aim, rgb(0, 255, 255));
                }
                if prog_res.dodge_active {
                    if let (Some(aim), Some(aim_use)) = (prog_res.aim_point, prog_res.aim_point_use)
                    {
                        draw_line(aim, aim_use, rgb(255, 255, 0));
                    }
                }
                if let Some(aim_use) = prog_res.aim_point_use {
                    draw_text!(
                        aim_use + vec2(0.0, 20.0),
                        rgb(255, 0, 0),
                        "Intercept: {:.2}s",
                        prog_res.t_go_use
                    );
                }
            }

            if let Some(impact_pos) = att_res.target_pos_at_impact {
                draw_diamond(impact_pos, 16.0, rgb(255, 0, 0));
            }

            // Proximity fuse detonation check visualization
            let t_bullet = self.proximity_ticks * TICK_LENGTH;
            let p_enemy_fuse =
                target.position_at(snapshot.current_tick + self.proximity_ticks.round() as u32);
            let bullet_pos_at_fuse = p + snapshot.velocity * t_bullet;
            draw_polygon(
                bullet_pos_at_fuse,
                1000.0 * t_bullet,
                16,
                0.0,
                rgb(255, 0, 128),
            ); // Pink circle
            draw_square(p_enemy_fuse, 8.0, rgb(255, 0, 128));
            draw_line(bullet_pos_at_fuse, p_enemy_fuse, rgb(255, 0, 128));

            // Terminal turn values and status comparison text
            if let (Some(t_exp), Some(t_turn)) =
                (att_res.time_until_explosion, att_res.turn_time_with_buffer)
            {
                draw_text!(
                    p + vec2(0.0, -40.0),
                    if att_res.is_terminal {
                        rgb(255, 0, 0)
                    } else {
                        rgb(200, 200, 200)
                    },
                    "t_exp: {:.2}s / t_turn: {:.2}s",
                    t_exp,
                    t_turn
                );
            }

            // Terminal turn details (aim vector, explosion point, target pos at explosion)
            if att_res.is_terminal {
                if let (Some(p_exp), Some(p_t_exp)) = (att_res.p_explode, att_res.p_target_explode)
                {
                    draw_diamond(p_exp, 15.0, rgb(255, 0, 0));
                    draw_text!(p_exp + vec2(0.0, -20.0), rgb(255, 0, 0), "Detonation Point");
                    draw_line(p, p_exp, rgb(255, 0, 0));

                    draw_square(p_t_exp, 15.0, rgb(0, 255, 255));
                    draw_text!(
                        p_t_exp + vec2(0.0, 20.0),
                        rgb(0, 255, 255),
                        "Target at Detonation"
                    );

                    draw_line(p_exp, p_t_exp, rgb(255, 255, 0));
                }
            }

            // Proportional Navigation acceleration visualizer
            if let Some(a_pn) = prog_res.a_pn {
                draw_line(p, p + a_pn, rgb(128, 0, 255)); // Purple for PN acceleration
                draw_text!(
                    p + a_pn + vec2(0.0, 10.0),
                    rgb(128, 0, 255),
                    "a_pn: {:.1}",
                    a_pn.length()
                );
            }
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct TargetSnapshot {
    pub position: Vec2,
    pub velocity: Vec2,
    pub class: Class,
    pub last_scanned: u32,
}

impl TargetSnapshot {
    pub fn position_at(&self, tick: u32) -> Vec2 {
        let dt = (tick as i64 - self.last_scanned as i64) as f64 * TICK_LENGTH;
        self.position + self.velocity * dt
    }

    pub fn velocity_at(&self, _tick: u32) -> Vec2 {
        self.velocity
    }

    pub fn to_kinematic_state(&self) -> KinematicState {
        KinematicState::new(
            self.class,
            self.position,
            self.velocity,
            vec2(0.0, 0.0),
            self.last_scanned,
        )
    }
}

#[derive(Clone, Copy, Debug)]
pub struct StateSnapshot {
    pub position: Vec2,
    pub velocity: Vec2,
    pub heading: f64,
    pub angular_velocity: f64,
    pub fuel: f64,
    pub current_tick: u32,
    pub max_forward_acceleration: f64,
    pub max_lateral_acceleration: f64,
    pub max_angular_acceleration: f64,
    pub target: Option<TargetSnapshot>,
    pub cruise_point: Option<Vec2>,
    pub has_entered_nez: bool,
    pub dodge_sign: f64,
}

impl StateSnapshot {
    pub fn to_kinematic_state(&self) -> KinematicState {
        KinematicState::new(
            Class::Missile,
            self.position,
            self.velocity,
            vec2(0.0, 0.0),
            self.current_tick,
        )
    }
}

#[derive(Clone, Copy, Debug)]
pub struct ProgradeResult {
    pub prograde: Option<Vec2>,
    pub cruise_point: Option<Vec2>,
    pub can_defeat: bool,
    pub nez_metric_passed: bool,
    pub dodge_active: bool,
    pub target_to_use: Option<TargetSnapshot>,
    pub aim_point: Option<Vec2>,
    pub aim_point_use: Option<Vec2>,
    pub t_go: f64,
    pub t_go_use: f64,
    pub v_c_use: f64,
    pub a_pn: Option<Vec2>,
}

pub fn check_detonation_condition(snapshot: &StateSnapshot, proximity_ticks: f64) -> bool {
    if let Some(target) = snapshot.target {
        let t_bullet = proximity_ticks * TICK_LENGTH;
        let p_enemy_fuse =
            target.position_at(snapshot.current_tick + proximity_ticks.round() as u32);
        let vec_bullet_rel = p_enemy_fuse - snapshot.position - snapshot.velocity * t_bullet;
        vec_bullet_rel.length() <= 1000.0 * t_bullet
    } else {
        false
    }
}

pub fn estimate_t_go(snapshot: &StateSnapshot, target: &TargetSnapshot, is_cruising: bool) -> f64 {
    let r = target.position_at(snapshot.current_tick) - snapshot.position;
    let r_len = r.length();
    if r_len < 1e-6 {
        return 0.0;
    }
    let dir = r / r_len;
    let us_kin = snapshot.to_kinematic_state();
    let v_max = crate::control::max_achievable_velocity(&us_kin, dir, snapshot.fuel)
        .unwrap_or(snapshot.velocity);
    let target_vel = target.velocity_at(snapshot.current_tick);
    let v_c_after = -(target_vel - v_max).dot(dir);

    let v_rel = target_vel - snapshot.velocity;
    let v_c = -v_rel.dot(dir);

    let possible_enemy_dv = if v_c > 0.0 {
        let t_intercept = r_len / v_c;
        let base_dv = t_intercept * target.class.default_stats().max_forward_acceleration;
        let boost_dv = (t_intercept / 10.0).ceil() * 100.0;
        base_dv + boost_dv
    } else {
        0.0
    };
    let fuel_economy = v_c >= v_c_after && snapshot.fuel < possible_enemy_dv;

    let fwd_budget = if fuel_economy {
        0.0
    } else {
        snapshot.max_forward_acceleration
    };

    let target_kin = target.to_kinematic_state();

    let t_go = if let Some(mei) = minimum_effort_intercept(&us_kin, &target_kin, fwd_budget) {
        mei.constant_velocity.t_go
    } else {
        let v_c_clamped = v_c
            .max(if is_cruising {
                v_c_after
            } else {
                snapshot.velocity.length()
            })
            .max(0.1);
        r_len / v_c_clamped
    };

    t_go
}

pub fn calculate_nez_metric(
    snapshot: &StateSnapshot,
    target: &TargetSnapshot,
    aim_point: Vec2,
    t_go: f64,
) -> f64 {
    let enemy_stats = target.class.default_stats();
    let a_enemy_base = enemy_stats
        .max_forward_acceleration
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

    let p_intercept_missile = snapshot.position + snapshot.velocity * t_go;
    let p_intercept_target =
        target.position_at(snapshot.current_tick + (t_go / TICK_LENGTH).round() as u32);
    let error_vector = p_intercept_target - p_intercept_missile;

    let r_aim = aim_point - snapshot.position;
    let dir_aim = if r_aim.length() > 1e-6 {
        r_aim.normalize()
    } else {
        vec2(snapshot.heading.cos(), snapshot.heading.sin())
    };
    let perp_aim = vec2(-dir_aim.y, dir_aim.x);
    let lateral_error = error_vector.dot(perp_aim).abs();

    let our_max_acceleration_sum = (snapshot.max_forward_acceleration * t_go).min(snapshot.fuel);
    let our_displacement = 0.5 * our_max_acceleration_sum * t_go;

    const SPARE_DV_REQUIRED: f64 = 150.0;
    our_displacement - (enemy_displacement + lateral_error) - SPARE_DV_REQUIRED
}

pub fn determine_prograde(
    snapshot: &StateSnapshot,
    pn_gain: f64,
    pn_min_vc: f64,
) -> ProgradeResult {
    // 1. Resolve/Establish cruise point if we have a target or already have a cruise point
    let cruise_point = if let Some(cp) = snapshot.cruise_point {
        Some(cp)
    } else if let Some(target) = snapshot.target {
        let target_pos = target.position_at(snapshot.current_tick);
        let target_vel = target.velocity_at(snapshot.current_tick);
        let vel_len = target_vel.length();
        let dir_course = if vel_len > 1e-6 {
            target_vel / vel_len
        } else {
            vec2(1.0, 0.0)
        };
        let offset_dist = rand(-2000.0, 2000.0);
        Some(target_pos + dir_course * offset_dist)
    } else {
        None
    };

    if snapshot.target.is_none() {
        if let Some(cp) = cruise_point {
            let diff = cp - snapshot.position;
            let prograde = if diff.length() > 1e-6 {
                diff.normalize()
            } else {
                vec2(snapshot.heading.cos(), snapshot.heading.sin())
            };
            return ProgradeResult {
                prograde: Some(prograde),
                cruise_point: Some(cp),
                can_defeat: false,
                nez_metric_passed: false,
                dodge_active: false,
                target_to_use: None,
                aim_point: None,
                aim_point_use: None,
                t_go: 0.0,
                t_go_use: 0.0,
                v_c_use: 0.0,
                a_pn: None,
            };
        } else {
            // No cruise point and no target
            return ProgradeResult {
                prograde: None,
                cruise_point: None,
                can_defeat: false,
                nez_metric_passed: false,
                dodge_active: false,
                target_to_use: None,
                aim_point: None,
                aim_point_use: None,
                t_go: 0.0,
                t_go_use: 0.0,
                v_c_use: 0.0,
                a_pn: None,
            };
        }
    }

    let target = snapshot.target.unwrap();
    let target_kinematic = target.to_kinematic_state();

    // 2. Decide if we can defeat the target
    let t_go = estimate_t_go(snapshot, &target, true);
    let aim_point = target.position_at(snapshot.current_tick + (t_go / TICK_LENGTH).round() as u32);

    let nez_metric = calculate_nez_metric(snapshot, &target, aim_point, t_go);
    let nez_metric_passed = nez_metric >= 0.0;
    let can_defeat = nez_metric_passed || t_go < 10.0;

    if !can_defeat {
        // If we cannot defeat, prograde is toward the established cruise point.
        let cp = cruise_point.unwrap();
        let diff = cp - snapshot.position;
        let prograde = if diff.length() > 1e-6 {
            diff.normalize()
        } else {
            vec2(snapshot.heading.cos(), snapshot.heading.sin())
        };
        return ProgradeResult {
            prograde: Some(prograde),
            cruise_point: Some(cp),
            can_defeat: false,
            nez_metric_passed: false,
            dodge_active: false,
            target_to_use: Some(target),
            aim_point: Some(aim_point),
            aim_point_use: Some(aim_point),
            t_go,
            t_go_use: t_go,
            v_c_use: 0.0,
            a_pn: None,
        };
    }

    // 3. Dodging logic
    let mut extra_dv = 0.0;
    let us_kin = snapshot.to_kinematic_state();
    let r = target.position_at(snapshot.current_tick) - snapshot.position;
    let r_len = r.length();
    let dir_target = if r_len > 1e-6 {
        r / r_len
    } else {
        vec2(1.0, 0.0)
    };
    let v_max = crate::control::max_achievable_velocity(&us_kin, dir_target, snapshot.fuel)
        .unwrap_or(snapshot.velocity);
    let target_vel = target.velocity_at(snapshot.current_tick);
    let v_c_after = -(target_vel - v_max).dot(dir_target);

    let fuel_economy_orig = {
        let v_rel = target_vel - snapshot.velocity;
        let v_c = -v_rel.dot(dir_target);
        let possible_enemy_dv = if v_c > 0.0 {
            let t_intercept = r_len / v_c;
            let base_dv = t_intercept * target.class.default_stats().max_forward_acceleration;
            let boost_dv = (t_intercept / 10.0).ceil() * 100.0;
            base_dv + boost_dv
        } else {
            0.0
        };
        v_c >= v_c_after && snapshot.fuel < possible_enemy_dv
    };

    let fwd_budget = if fuel_economy_orig {
        0.0
    } else {
        snapshot.max_forward_acceleration
    };
    if let Some(mei) = minimum_effort_intercept(&us_kin, &target_kinematic, fwd_budget) {
        let worst_case_fuel = mei
            .worst_case_positive
            .fuel_consumed
            .max(mei.worst_case_negative.fuel_consumed);
        extra_dv = snapshot.fuel - worst_case_fuel;
    }

    let dodge_active = snapshot.has_entered_nez && extra_dv >= N && t_go > M;
    let mut target_to_use = target;
    if dodge_active {
        let r = target.position - snapshot.position;
        let r_len = r.length();
        let perp_dir = if r_len > 1e-6 {
            vec2(-r.y, r.x) / r_len
        } else {
            vec2(-snapshot.heading.sin(), snapshot.heading.cos())
        };
        let offset = perp_dir * (snapshot.dodge_sign * (N / 2.0));
        target_to_use.position += offset;
    }

    // Recalculate tracking using target_to_use
    let target_pos_use = target_to_use.position_at(snapshot.current_tick);
    let target_vel_use = target_to_use.velocity_at(snapshot.current_tick);
    let r_use = target_pos_use - snapshot.position;
    let r_len_use = r_use.length();
    let v_rel_use = target_vel_use - snapshot.velocity;

    let numerator_use = r_use.x * v_rel_use.y - r_use.y * v_rel_use.x;
    let denominator_use = r_use.dot(r_use);
    let los_rate_use = if denominator_use > 1e-6 {
        numerator_use / denominator_use
    } else {
        0.0
    };

    let v_c_use = -v_rel_use.dot(r_use) / r_len_use.max(1e-6);
    let t_go_use = estimate_t_go(snapshot, &target_to_use, false);
    let aim_point_use =
        target_to_use.position_at(snapshot.current_tick + (t_go_use / TICK_LENGTH).round() as u32);

    let e_perp_use = vec2(-r_use.y, r_use.x) / r_len_use.max(1e-6);
    let a_pn = pn_gain * v_c_use.max(pn_min_vc) * los_rate_use * e_perp_use;

    let prograde = (snapshot.velocity + a_pn).normalize();
    ProgradeResult {
        prograde: Some(prograde),
        cruise_point,
        can_defeat: true,
        nez_metric_passed,
        dodge_active,
        target_to_use: Some(target_to_use),
        aim_point: Some(aim_point),
        aim_point_use: Some(aim_point_use),
        t_go,
        t_go_use,
        v_c_use,
        a_pn: Some(a_pn),
    }
}

pub fn calculate_min_lateral_thrust(v: Vec2, p: Vec2, max_lat: f64) -> Vec2 {
    let v_len = v.length();
    if v_len < 1e-6 {
        return vec2(0.0, 0.0);
    }
    let dot = p.dot(v);
    if dot > 1e-6 {
        let a_lat = (p * (v_len * v_len / dot) - v) / TICK_LENGTH;
        if a_lat.length() > max_lat {
            a_lat.normalize() * max_lat
        } else {
            a_lat
        }
    } else {
        let v_hat = v / v_len;
        let p_perp = p - p.dot(v_hat) * v_hat;
        if p_perp.length() > 1e-6 {
            p_perp.normalize() * max_lat
        } else {
            vec2(-v.y, v.x).normalize() * max_lat
        }
    }
}

pub fn determine_thrust(
    snapshot: &StateSnapshot,
    prograde: Option<Vec2>,
    fuel_limit: f64,
) -> Option<Vec2> {
    let prograde = prograde?;

    let available_dv = (snapshot.fuel - fuel_limit).max(0.0);
    let us_kin = snapshot.to_kinematic_state();

    if let Some(v_desired) =
        crate::control::max_achievable_velocity(&us_kin, prograde, available_dv)
    {
        let thrust_dir =
            crate::control::match_velocity_thrust_heading(snapshot.velocity, v_desired);
        if let Some(dv_dir) = thrust_dir {
            let heading_vec = vec2(snapshot.heading.cos(), snapshot.heading.sin());
            let dp_parallel = dv_dir.dot(heading_vec);
            let dp_perp = (dv_dir - dp_parallel * heading_vec).length();

            let a_max_p = snapshot.max_forward_acceleration * dp_parallel.abs()
                + snapshot.max_lateral_acceleration * dp_perp;

            let thrust_mag = 0.95 * a_max_p;
            let max_vel_change = thrust_mag * TICK_LENGTH;
            let dv = v_desired - snapshot.velocity;

            let thrust = if dv.length() < max_vel_change {
                dv / TICK_LENGTH
            } else {
                dv_dir * thrust_mag
            };
            Some(thrust)
        } else {
            Some(vec2(0.0, 0.0))
        }
    } else {
        Some(calculate_min_lateral_thrust(
            snapshot.velocity,
            prograde,
            snapshot.max_lateral_acceleration,
        ))
    }
}

#[derive(Clone, Copy, Debug)]
pub struct AttitudeResult {
    pub desired_heading: f64,
    pub target_omega: Option<f64>,
    pub is_terminal: bool,
    pub target_pos_at_impact: Option<Vec2>,
    pub time_until_explosion: Option<f64>,
    pub turn_time_with_buffer: Option<f64>,
    pub explode_heading: Option<f64>,
    pub p_explode: Option<Vec2>,
    pub p_target_explode: Option<Vec2>,
}

pub fn determine_attitude(
    snapshot: &StateSnapshot,
    thrust: Option<Vec2>,
    prograde: Option<Vec2>,
    proximity_ticks: f64,
    turn_safety_buffer_ticks: f64,
) -> AttitudeResult {
    if let Some(target) = snapshot.target {
        let target_pos = target.position_at(snapshot.current_tick);
        let target_vel = target.velocity_at(snapshot.current_tick);
        let r = target_pos - snapshot.position;
        let v_rel = target_vel - snapshot.velocity;

        let t_bullet = proximity_ticks * TICK_LENGTH;
        let bullet_range = 1000.0 * t_bullet;
        let a_coef = v_rel.dot(v_rel);
        let b_coef = 2.0 * r.dot(v_rel);
        let c_coef = r.dot(r) - bullet_range.powi(2);
        let discriminant = b_coef * b_coef - 4.0 * a_coef * c_coef;

        let target_kin = target.to_kinematic_state();
        let us_kin = snapshot.to_kinematic_state();
        let t_go_fallback = if let Some(mei) =
            minimum_effort_intercept(&us_kin, &target_kin, snapshot.max_forward_acceleration)
        {
            mei.constant_velocity.t_go
        } else {
            let v_c = -v_rel.dot(r) / r.length().max(1e-6);
            r.length() / v_c.max(0.1)
        };

        let t_total = if discriminant >= 0.0 && a_coef > 1e-6 {
            let sol1 = (-b_coef - discriminant.sqrt()) / (2.0 * a_coef);
            let sol2 = (-b_coef + discriminant.sqrt()) / (2.0 * a_coef);
            if sol1 >= t_bullet {
                sol1
            } else if sol2 >= t_bullet {
                sol2
            } else {
                t_bullet
            }
        } else {
            t_go_fallback.max(t_bullet)
        };

        let time_until_explosion = (t_total - t_bullet).max(0.0);

        let t_exp = time_until_explosion;
        let p_explode = snapshot.position + t_exp * snapshot.velocity;
        let p_target_explode =
            target.position_at(snapshot.current_tick + (t_exp / TICK_LENGTH).round() as u32);
        let v_target_explode =
            target.velocity_at(snapshot.current_tick + (t_exp / TICK_LENGTH).round() as u32);
        let r_explode = p_target_explode - p_explode;
        let v_rel_explode = v_target_explode - snapshot.velocity;

        let vec_aim = r_explode + v_rel_explode * t_bullet;
        let explode_heading = vec_aim.angle();

        let diff = angle_diff(snapshot.heading, explode_heading);
        let omega = snapshot.angular_velocity;
        let a = snapshot.max_angular_acceleration.max(1.0);

        let time_to_stop = if omega * diff < 0.0 {
            omega.abs() / a
        } else {
            0.0
        };
        let angle_to_stop = 0.5 * omega.powi(2) / a;
        let remaining_angle = (diff.abs()
            + if omega * diff < 0.0 {
                angle_to_stop
            } else {
                -angle_to_stop
            })
        .max(0.0);
        let time_remaining_turn = 2.0 * (remaining_angle / a).sqrt();
        let turn_time = time_to_stop + time_remaining_turn;

        let safety_buffer = turn_safety_buffer_ticks * TICK_LENGTH;
        let turn_time_with_buffer = turn_time + safety_buffer;

        let is_terminal =
            time_until_explosion <= turn_time_with_buffer || time_until_explosion < 0.5;
        let desired_heading = if is_terminal {
            explode_heading
        } else if let Some(t) = thrust {
            if t.length() > 1e-6 {
                t.angle()
            } else if let Some(p) = prograde {
                p.angle()
            } else {
                snapshot.heading
            }
        } else if let Some(p) = prograde {
            p.angle()
        } else {
            snapshot.heading
        };
        let target_omega = if is_terminal { Some(0.0) } else { None };

        return AttitudeResult {
            desired_heading,
            target_omega,
            is_terminal,
            target_pos_at_impact: Some(
                target.position_at(snapshot.current_tick + (t_total / TICK_LENGTH).round() as u32),
            ),
            time_until_explosion: Some(time_until_explosion),
            turn_time_with_buffer: Some(turn_time_with_buffer),
            explode_heading: Some(explode_heading),
            p_explode: Some(p_explode),
            p_target_explode: Some(p_target_explode),
        };
    }

    let desired_heading = if let Some(t) = thrust {
        if t.length() > 1e-6 {
            t.angle()
        } else if let Some(p) = prograde {
            p.angle()
        } else {
            snapshot.heading
        }
    } else if let Some(p) = prograde {
        p.angle()
    } else {
        snapshot.heading
    };

    AttitudeResult {
        desired_heading,
        target_omega: None,
        is_terminal: false,
        target_pos_at_impact: None,
        time_until_explosion: None,
        turn_time_with_buffer: None,
        explode_heading: None,
        p_explode: None,
        p_target_explode: None,
    }
}

pub fn should_boost(snapshot: &StateSnapshot, thrust: Vec2, fuel_limit: f64) -> bool {
    let fuel_economy = snapshot.fuel < fuel_limit;
    if fuel_economy {
        return false;
    }
    let aimed_correctly = if thrust.length() > 0.0 {
        angle_diff(snapshot.heading, thrust.angle()).abs() < 5.0f64.to_radians()
    } else {
        false
    };
    thrust.length() > 100.0 && aimed_correctly
}

pub struct MissileAimer {
    pub available_dv: f64,
    pub launch_speed: f64,
}

impl MissileAimer {
    pub fn new(available_dv: f64, launch_speed: f64) -> Self {
        Self {
            available_dv,
            launch_speed,
        }
    }

    pub fn calculate_fire_direction(
        &self,
        target: &KinematicState,
        us: &KinematicState,
        current_tick: u32,
    ) -> Option<(Vec2, f64)> {
        let target_pos = target_position_at(target, current_tick);
        let r = target_pos - us.position;
        let r_len = r.length();
        let dir_est = if r_len > 1e-6 {
            r / r_len
        } else {
            Vec2::new(
                us.heading.unwrap_or(0.0).cos(),
                us.heading.unwrap_or(0.0).sin(),
            )
        };

        // 1. Initial velocity estimation assuming launch in dir_est
        let v_launch_est = us.velocity + dir_est * self.launch_speed;
        let missile_kin = KinematicState::new(
            Class::Missile,
            us.position,
            v_launch_est,
            Vec2::new(0.0, 0.0),
            current_tick,
        );
        let v_max_est =
            crate::control::max_achievable_velocity(&missile_kin, dir_est, self.available_dv)
                .unwrap_or(v_launch_est);
        let v_speed = v_max_est.length().max(0.1);

        // 2. Predict lead intercept time and direction using absolute missile speed
        let (t_go, dir_aim) = crate::control::predict_lead(
            us.position,
            Vec2::new(0.0, 0.0),
            v_speed,
            target.position_at(current_tick),
            target.velocity_at(current_tick),
            target.acceleration,
        )?;

        // 3. Compute actual optimal launch velocity and burn required
        let v_launch_opt = us.velocity + dir_aim * self.launch_speed;
        let missile_kin_opt = KinematicState::new(
            Class::Missile,
            us.position,
            v_launch_opt,
            Vec2::new(0.0, 0.0),
            current_tick,
        );
        let v_final =
            crate::control::max_achievable_velocity(&missile_kin_opt, dir_aim, self.available_dv)
                .unwrap_or(v_launch_opt);
        let v_diff = v_final - us.velocity;

        if v_diff.length() > 1e-6 {
            Some((v_diff.normalize(), t_go))
        } else {
            Some((dir_aim, t_go))
        }
    }
}

impl AimAt for MissileAimer {
    fn aim_at(&self, target: &KinematicState, us: &KinematicState) -> Option<(Vec2, f64)> {
        // Compute firing direction for current tick
        let (fire_dir, _) = self.calculate_fire_direction(target, us, current_tick())?;

        // Extrapolate state to next tick to compute omega
        let target_next = KinematicState::new(
            target.class,
            target.position_at(current_tick() + 1),
            target.velocity_at(current_tick() + 1),
            target.acceleration,
            current_tick() + 1,
        );

        let us_next = KinematicState::new(
            us.class,
            us.position + us.velocity * TICK_LENGTH,
            us.velocity + us.acceleration * TICK_LENGTH,
            us.acceleration,
            current_tick() + 1,
        );

        let mut omega = 0.0;
        if let Some((fire_dir_next, _)) =
            self.calculate_fire_direction(&target_next, &us_next, current_tick() + 1)
        {
            omega = angle_diff(fire_dir.angle(), fire_dir_next.angle()) / TICK_LENGTH;
        }

        Some((fire_dir, omega))
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

    let max_fwd_acc = if cfg!(test) {
        250.0
    } else {
        max_forward_acceleration()
    };
    let max_lat_acc = if cfg!(test) {
        300.0
    } else {
        max_forward_acceleration()
    };
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

    let p_reach_min =
        enemy.position + enemy.velocity * t_go + 0.5 * a_e_reach_min * t_go * t_go * w0;
    let p_reach_max =
        enemy.position + enemy.velocity * t_go + 0.5 * a_e_reach_max * t_go * t_go * w0;

    draw_line(
        p_reach_min - u0 * (tick_len / 2.0),
        p_reach_min + u0 * (tick_len / 2.0),
        rgb(0, 255, 0),
    );
    draw_line(
        p_reach_max - u0 * (tick_len / 2.0),
        p_reach_max + u0 * (tick_len / 2.0),
        rgb(0, 255, 0),
    );

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
