use oort_api::prelude::*;
use crate::control::quick_turn_with_target_omega;
use crate::missile::TargetTelemetry;
use crate::radar::{RadarController, DefaultScanSliceGenerator};
use crate::radio::SecureRadio;
use crate::physics::KinematicState;
use crate::aim::{AimAt, GunAimer};

const GUN_AIMER: GunAimer = GunAimer::new(Vec2 { x: 20.0, y: 0.0 }, 1000.0);

pub struct Fighter {
    radar_controller: RadarController,
    missile_sender: crate::missile::MissileRadioSender,
    fighter_radio: SecureRadio,
    
    // Fighter coordination and target tracking fields
    fighter_target_id: Option<u32>,
    fighter_msgs_received: u32,
    fighter_last_known_target_pos: Option<Vec2>,
    fighter_last_known_target_vel: Option<Vec2>,

    // Orbit and movement fields
    orbit_direction: f64,
    current_period_ticks: u32,
    last_orbit_direction_change_tick: u32,
    num_direction_changes: u32,
}

impl Fighter {
    pub fn new(missile_radio: SecureRadio, fighter_radio: SecureRadio) -> Self {
        let mut rc = RadarController::new();
        // Double the base scan range from 10000.0 to 20000.0
        rc.slice_generator = Box::new(DefaultScanSliceGenerator::new(0.6, 20000.0));

        Fighter {
            radar_controller: rc,
            missile_sender: crate::missile::MissileRadioSender::new(missile_radio),
            fighter_radio,
            fighter_target_id: None,
            fighter_msgs_received: 0,
            fighter_last_known_target_pos: None,
            fighter_last_known_target_vel: None,
            orbit_direction: if rand(0.0, 1.0) < 0.5 { 1.0 } else { -1.0 },
            current_period_ticks: ((rand(7.0, 13.0) / 2.0) / TICK_LENGTH).round() as u32,
            last_orbit_direction_change_tick: 0,
            num_direction_changes: 0,
        }
    }

    pub fn tick(&mut self) {
        debug!("Fighter ID: {}", id());
        debug!("Position: {:?}", position());

        if let Some(tid) = self.fighter_target_id {
            self.radar_controller.priority_targets = vec![tid];
        }

        let target_idx = (id() as i32) - 2; // Fighter 2 -> 0, Fighter 3 -> 1, Fighter 4 -> 2
        let mut received_msg = None;
        if target_idx >= 0 && target_idx < 3 {
            let received = self.fighter_radio.receive();
            if let Some(payload) = received {
                self.fighter_msgs_received += 1;
                let telemetry = TargetTelemetry::deserialize(&payload);
                debug!("Fighter {} received radio message #{}: pos={:?}, target_idx={}, target_idx+1={}", id(), self.fighter_msgs_received, telemetry.position, target_idx, target_idx + 1);
                if self.fighter_msgs_received == (target_idx + 1) as u32 {
                    received_msg = Some(telemetry);
                } else {
                    // Treat as a virtual radar ping to update our contact database
                    self.radar_controller.add_radio_ping(telemetry);
                }
            }
        }

        self.radar_controller.update();
        self.missile_sender.send_missile_contact(self.radar_controller.contacts());

        // 3. Get contacts
        let contacts = self.radar_controller.contacts().to_vec();

        // 4. Update target tracking
        let received_assignment = self.fighter_msgs_received >= (target_idx + 1) as u32;
        if !received_assignment {
            self.fighter_radio.prepare_receive();
        }

        if received_assignment {
            if let Some(telemetry) = received_msg {
                self.fighter_target_id = Some(self.radar_controller.add_radio_ping(telemetry));
                debug!("Fighter {} acquired assigned target via radio at {:?}", id(), telemetry.position);
            }

            // If target is dead/None, select closest enemy fighter from radar contacts
            if self.fighter_target_id.is_none() || !self.radar_controller.has_contact(self.fighter_target_id.unwrap()) {
                let mut best_c = None;
                let mut min_d = f64::MAX;
                for c in contacts.iter().filter(|c| c.class == Class::Fighter) {
                    let d = c.current_position().distance(position());
                    if d < min_d {
                        min_d = d;
                        best_c = Some(c.clone());
                    }
                }
                if let Some(c) = best_c {
                    self.fighter_target_id = Some(c.id);
                    debug!("Fighter {} selected closest target: ID {}", id(), c.id);
                }
            }

            // 4.5. Print target diagnostics
            if let Some(tid) = self.fighter_target_id {
                debug!("Fighter {} tracking target ID: {}", id(), tid);
                if let Some(contact) = self.radar_controller.get_contact(tid) {
                    let ticks_since_tracked = current_tick() - contact.last_scanned;
                    debug!("  Contact {} last scanned {} ticks ago (provisional={})", tid, ticks_since_tracked, contact.provisional);
                    debug!("  Position: {:?}", contact.current_position());
                    debug!("  Velocity: {:?}", contact.current_velocity());
                    // Draw a magenta circle at the contact position
                    draw_polygon(contact.current_position(), 50.0, 16, 0.0, rgb(255, 0, 255));
                } else {
                    debug!("  Contact {} NOT found in radar database!", tid);
                }
            } else {
                debug!("Fighter {} tracking target ID: None", id());
            }

            // 4. Movement & Aiming/Firing
            if let Some(contact) = self.fighter_target_id.and_then(|tid| self.radar_controller.get_contact(tid)) {
                let (target_pos, target_vel) = (contact.current_position(), contact.current_velocity());
                let target_accel = contact.acceleration;
                
                // Orbit target at 3km
                draw_line(position(), target_pos, rgb(0, 255, 255));

                let r_orbit = 3000.0;
                let rel_pos = position() - target_pos;
                let rel_vel = velocity() - target_vel;
                let d = rel_pos.length();
                let u = if d > 1.0 { rel_pos / d } else { vec2(1.0, 0.0) };

                let speed = rel_vel.length();
                let max_acc = max_forward_acceleration();

                let margin = 0.90; // Reserve 10% of acceleration for radial/tangential control
                let target_speed = (r_orbit * max_acc * margin).sqrt();

                // Periodic orbit direction change (randomly between 7.0 and 13.0 seconds)
                let ticks_since_change = current_tick() - self.last_orbit_direction_change_tick;
                if ticks_since_change >= self.current_period_ticks {
                    self.orbit_direction = -self.orbit_direction;
                    let t_seconds = rand(13.0, 17.0);
                    self.current_period_ticks = (t_seconds / TICK_LENGTH).round() as u32;
                    self.last_orbit_direction_change_tick = current_tick();
                    self.num_direction_changes += 1;
                    debug!("Orbit direction flipped to {}. Next change in {:.1}s", self.orbit_direction, t_seconds);
                }

                // Calculate desired acceleration for movement based on zone
                let mut acc_cmd;
                if d < r_orbit - 200.0 {
                    // Inside: burn at 45 degrees outward relative to the radial direction
                    let burn_dir = u.rotate(self.orbit_direction * (TAU / 8.0));
                    let desired_vel = burn_dir * target_speed;
                    acc_cmd = 1.0 * (desired_vel - rel_vel);
                } else if d > r_orbit + 200.0 {
                    // Outside: drive towards the circle on a tangent line
                    let phi = (r_orbit / d).asin();
                    let tangent_dir = (-u).rotate(-self.orbit_direction * phi);
                    let desired_vel = tangent_dir * target_speed;
                    acc_cmd = 1.0 * (desired_vel - rel_vel);
                } else {
                    // On the circle: standard orbit control
                    let t = vec2(-u.y, u.x) * self.orbit_direction;
                    let centripetal_needed = speed.powi(2) / r_orbit;

                    // Radial control: regulate distance to r_orbit
                    let e_r = d - r_orbit;
                    let v_r = rel_vel.dot(u);
                    let kp_r = 0.5;
                    let kd_r = 1.0;
                    let radial_accel_mag = centripetal_needed + kp_r * e_r + kd_r * v_r;
                    
                    // Tangential control: regulate speed to target_speed
                    let v_t = rel_vel.dot(t);
                    let kp_t = 0.5;
                    let tangential_accel_mag = kp_t * (target_speed - v_t);

                    acc_cmd = -radial_accel_mag * u + tangential_accel_mag * t;
                }

                // Add feedforward target acceleration
                acc_cmd += target_accel;

                if acc_cmd.length() > max_acc {
                    acc_cmd = acc_cmd.normalize() * max_acc;
                }

                // Apply acceleration command for movement
                accelerate(acc_cmd);

                // 5. Aim and Shoot Cannon
                let to_target = target_pos - position();
                let us_state = KinematicState::self_state();
                let aim_res = GUN_AIMER.aim_at(&contact.kinematic, &us_state);

                let mut target_angle_now = None;
                let (target_angle, target_omega) = if let Some((lead_dir, omega)) = aim_res {
                    let angle = lead_dir.angle();
                    target_angle_now = Some(angle);

                    // Draw line to predicted target position
                    let time_to_impact = (contact.current_position() - position()).length() / GUN_AIMER.bullet_speed;
                    let p_e = target_pos + time_to_impact * target_vel + 0.5 * target_accel * time_to_impact * (time_to_impact + TICK_LENGTH);
                    draw_line(position(), p_e, rgb(255, 255, 0));

                    (angle, omega)
                } else {
                    (to_target.angle(), 0.0)
                };

                quick_turn_with_target_omega(target_angle, target_omega);

                // Fire main gun (slot 0) if cooldown <= 5 and aligned
                if let Some(angle_now) = target_angle_now {
                    let diff = angle_diff(heading(), angle_now);
                    if reload_ticks(0) <= 5 && diff.abs() < 0.15f64.to_radians() {
                        fire(0);
                    }
                }
            } else {
                // Hold station at geometric center / origin if no target
                let center = vec2(0.0, 0.0);
                let error = center - position();
                accelerate(0.1 * error - 0.6 * velocity());
            }

            // --- Dedicated Missile Telemetry and Firing ---
            let mut current_missile_target = None;
            if let Some(c) = contacts.iter().find(|c| c.class == Class::Fighter) {
                current_missile_target = Some((c.id, c.current_position(), c.current_velocity(), c.rssi as f32, c.class));
            } else if let Some(last_pos) = self.fighter_last_known_target_pos {
                let last_vel = self.fighter_last_known_target_vel.unwrap_or(Vec2::new(0.0, 0.0));
                let tid = self.fighter_target_id.unwrap_or(0);
                current_missile_target = Some((tid, last_pos, last_vel, 0.0f32, Class::Fighter));
            }

            if let Some((tid, m_pos, _m_vel, _m_rssi, _m_class)) = current_missile_target {
                if reload_ticks(1) == 0 {
                    self.missile_sender.fire_missile(1, tid);
                    debug!("Fighter {} fired missile at target {:?}", id(), m_pos);
                }
            }
        }
    }
}
