use oort_api::prelude::*;
use crate::missile::TargetTelemetry;
use crate::physics::KinematicState;

pub const RADIO_PING_SNR: f64 = 25.0;

fn target_rcs(class: Class) -> f64 {
    match class {
        Class::Fighter => 10.0,
        Class::Frigate => 30.0,
        Class::Cruiser => 40.0,
        Class::Missile => 0.1,
        Class::Torpedo => 0.3,
        Class::Target => 10.0,
        _ => 10.0,
    }
}

fn own_radar_properties() -> (f64, f64) {
    if cfg!(test) {
        return (100e3, 10.0);
    }
    match class() {
        Class::Fighter => (20e3, 5.0),
        Class::Frigate => (100e3, 10.0),
        Class::Cruiser => (200e3, 20.0),
        Class::Missile => (1e3, 3.0),
        Class::Torpedo => (10e3, 3.0),
        _ => (100e3, 10.0),
    }
}

fn clamped_tracking_width(
    contact: &Contact,
    d: f64,
    gate_radius: f64,
    next_pos_uncertainty: f64,
    tracking_width: f64,
) -> f64 {
    let d_back = d + contact.ci_mult() * next_pos_uncertainty;
    let (power, rx_xs) = own_radar_properties();
    let rcs = target_rcs(contact.class);
    let reliable_rssi = 1e-12; // -90 dBm
    let range_limited_width = (power * rcs * rx_xs) / (std::f64::consts::TAU * reliable_rssi * d_back.powi(4));
    (2.0 * gate_radius / d).min(range_limited_width).clamp(0.005, tracking_width)
}

fn is_within_range(contact: &Contact, d: f64) -> bool {
    let (power, rx_xs) = own_radar_properties();
    let rcs = target_rcs(contact.class);
    let reliable_rssi = 1e-12; // -90 dBm
    let max_range = ((power * rcs * rx_xs) / (0.005 * std::f64::consts::TAU * reliable_rssi)).powf(0.25);
    d <= max_range
}

fn is_within_reliable_range(class: Class, d: f64, slice_width: f64) -> bool {
    let (power, rx_xs) = own_radar_properties();
    let rcs = target_rcs(class);
    let reliable_rssi = 1e-12; // -90 dBm
    let max_range = ((power * rcs * rx_xs) / (slice_width * std::f64::consts::TAU * reliable_rssi)).powf(0.25);
    d <= max_range
}


#[derive(Clone, Debug)]
pub struct Contact {
    pub id: u32,
    pub kinematic: KinematicState,
    pub rssi: f64,
    pub snr: f64,
    pub pos_uncertainty: f64,
    pub vel_uncertainty: f64,
    pub radar_locked: bool,
    pub provisional: bool,
    pub tracking_retry_count: u32,
    pub confirmation_attempts: u32,
    pub unscanned_in_range_ticks: u32,
    pub p_cov_x: [[f64; 3]; 3],
    pub p_cov_y: [[f64; 3]; 3],
    pub prioritize_scan: bool,
    pub prev_scan_pos_uncertainty: Option<f64>,
    pub low_improvement_consecutive_scans: u32,
}

impl std::ops::Deref for Contact {
    type Target = KinematicState;
    fn deref(&self) -> &Self::Target {
        &self.kinematic
    }
}

impl std::ops::DerefMut for Contact {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.kinematic
    }
}

impl Contact {
    pub fn ci_mult(&self) -> f64 {
        match self.tracking_retry_count {
            0 => 2.58,
            1 => 3.29,
            _ => 3.89,
        }
    }

    pub fn initial_cov(pos_unc: f64, vel_unc: f64, class: Class) -> [[f64; 3]; 3] {
        let stats = class.default_stats();
        let mut max_acc = stats.max_forward_acceleration
            .max(stats.max_backward_acceleration)
            .max(stats.max_lateral_acceleration);
        if class == Class::Fighter || class == Class::Missile {
            max_acc += 100.0;
        }
        let sigma_a = max_acc;
        [
            [pos_unc.powi(2), 0.0, 0.0],
            [0.0, vel_unc.powi(2), 0.0],
            [0.0, 0.0, sigma_a.powi(2)],
        ]
    }

    pub fn position_at(&self, tick: u32) -> Vec2 {
        self.kinematic.position_at(tick)
    }

    pub fn velocity_at(&self, tick: u32) -> Vec2 {
        self.kinematic.velocity_at(tick)
    }

    pub fn pos_uncertainty_at(&self, tick: u32) -> f64 {
        let dt = tick.wrapping_sub(self.last_scanned) as f64 * TICK_LENGTH;
        if dt <= 0.0 {
            return self.pos_uncertainty;
        }
        let f02 = 0.5 * dt * (dt + TICK_LENGTH);
        
        let var_pred_x = self.p_cov_x[0][0] 
            + 2.0 * dt * self.p_cov_x[0][1] 
            + 2.0 * f02 * self.p_cov_x[0][2] 
            + dt.powi(2) * self.p_cov_x[1][1] 
            + 2.0 * dt * f02 * self.p_cov_x[1][2] 
            + f02.powi(2) * self.p_cov_x[2][2];

        let var_pred_y = self.p_cov_y[0][0] 
            + 2.0 * dt * self.p_cov_y[0][1] 
            + 2.0 * f02 * self.p_cov_y[0][2] 
            + dt.powi(2) * self.p_cov_y[1][1] 
            + 2.0 * dt * f02 * self.p_cov_y[1][2] 
            + f02.powi(2) * self.p_cov_y[2][2];

        // Process noise
        let stats = self.class.default_stats();
        let mut max_acc = stats.max_forward_acceleration
            .max(stats.max_backward_acceleration)
            .max(stats.max_lateral_acceleration);
        if self.class == Class::Fighter || self.class == Class::Missile {
            max_acc += 100.0;
        }
        let s = (0.5 * max_acc).powi(2);
        let q00 = s * dt.powi(5) / 20.0;

        let unc_x = (var_pred_x + q00).max(0.0).sqrt();
        let unc_y = (var_pred_y + q00).max(0.0).sqrt();
        unc_x.max(unc_y)
    }

    pub fn current_position(&self) -> Vec2 {
        self.position_at(current_tick())
    }

    pub fn current_velocity(&self) -> Vec2 {
        self.velocity_at(current_tick())
    }

    pub fn current_pos_uncertainty(&self) -> f64 {
        self.pos_uncertainty_at(current_tick())
    }

    pub fn predict_and_update(&mut self, current_t: u32, z_pos: Vec2, z_vel: Vec2, sigma_p: f64, sigma_v: f64) {
        let dt = current_t.wrapping_sub(self.last_scanned) as f64 * TICK_LENGTH;
        if dt <= 0.0 {
            return;
        }

        // Get process noise jerk spectral density S
        let stats = self.class.default_stats();
        let mut max_acc = stats.max_forward_acceleration
            .max(stats.max_backward_acceleration)
            .max(stats.max_lateral_acceleration);
        if self.class == Class::Fighter || self.class == Class::Missile {
            max_acc += 100.0;
        }
        let s = (0.5 * max_acc).powi(2);

        // Precompute transition matrix components for F(dt)
        let f02 = 0.5 * dt * (dt + TICK_LENGTH);

        // Process noise matrix Q(dt)
        let q00 = s * dt.powi(5) / 20.0;
        let q01 = s * dt.powi(4) / 8.0;
        let q02 = s * dt.powi(3) / 6.0;
        let q11 = s * dt.powi(3) / 3.0;
        let q12 = s * dt.powi(2) / 2.0;
        let q22 = s * dt;

        // Perform prediction and update for both X and Y
        let (pos_x, vel_x, acc_x, cov_x) = self.predict_and_update_dim(
            self.position.x, self.velocity.x, self.acceleration.x, self.p_cov_x,
            dt, f02, q00, q01, q02, q11, q12, q22,
            z_pos.x, z_vel.x, sigma_p, sigma_v
        );

        let (pos_y, vel_y, acc_y, cov_y) = self.predict_and_update_dim(
            self.position.y, self.velocity.y, self.acceleration.y, self.p_cov_y,
            dt, f02, q00, q01, q02, q11, q12, q22,
            z_pos.y, z_vel.y, sigma_p, sigma_v
        );

        // Save back
        self.position = Vec2::new(pos_x, pos_y);
        self.velocity = Vec2::new(vel_x, vel_y);
        self.acceleration = Vec2::new(acc_x, acc_y);
        self.p_cov_x = cov_x;
        self.p_cov_y = cov_y;
        self.last_scanned = current_t;
        self.pos_uncertainty = self.p_cov_x[0][0].sqrt().max(self.p_cov_y[0][0].sqrt());
        self.vel_uncertainty = self.p_cov_x[1][1].sqrt().max(self.p_cov_y[1][1].sqrt());
    }

    fn predict_and_update_dim(
        &self,
        pos: f64, vel: f64, acc: f64, p: [[f64; 3]; 3],
        dt: f64, f02: f64,
        q00: f64, q01: f64, q02: f64, q11: f64, q12: f64, q22: f64,
        z_pos: f64, z_vel: f64, sigma_p: f64, sigma_v: f64
    ) -> (f64, f64, f64, [[f64; 3]; 3]) {
        // 1. Predict state
        let pos_pred = pos + vel * dt + f02 * acc;
        let vel_pred = vel + acc * dt;
        let acc_pred = acc;

        // 2. Predict covariance: P_pred = F * P * F^T + Q
        let fp00 = p[0][0] + dt * p[0][1] + f02 * p[0][2];
        let fp01 = p[0][1] + dt * p[1][1] + f02 * p[1][2];
        let fp02 = p[0][2] + dt * p[1][2] + f02 * p[2][2];
        let fp10 = p[0][1] + dt * p[0][2];
        let fp11 = p[1][1] + dt * p[1][2];
        let fp12 = p[1][2] + dt * p[2][2];
        let fp20 = p[0][2];
        let fp21 = p[1][2];
        let fp22 = p[2][2];

        let mut p_pred = [[0.0; 3]; 3];
        p_pred[0][0] = fp00 + dt * fp01 + f02 * fp02 + q00;
        p_pred[0][1] = fp01 + dt * fp02 + q01;
        p_pred[0][2] = fp02 + q02;
        p_pred[1][0] = fp10 + dt * fp11 + f02 * fp12 + q01;
        p_pred[1][1] = fp11 + dt * fp12 + q11;
        p_pred[1][2] = fp12 + q12;
        p_pred[2][0] = fp20 + dt * fp21 + f02 * fp22 + q02;
        p_pred[2][1] = fp21 + dt * fp22 + q12;
        p_pred[2][2] = fp22 + q22;

        // Enforce symmetry
        p_pred[1][0] = p_pred[0][1];
        p_pred[2][0] = p_pred[0][2];
        p_pred[2][1] = p_pred[1][2];

        // 3. Kalman Gain Update
        let r_p = sigma_p.powi(2);
        let r_v = sigma_v.powi(2);
        
        let d = (p_pred[0][0] + r_p) * (p_pred[1][1] + r_v) - p_pred[0][1].powi(2);
        if d.abs() < 1e-12 {
            return (pos_pred, vel_pred, acc_pred, p_pred);
        }

        let k00 = (p_pred[0][0] * (p_pred[1][1] + r_v) - p_pred[0][1].powi(2)) / d;
        let k01 = (p_pred[0][1] * r_p) / d;
        let k10 = (p_pred[0][1] * r_v) / d;
        let k11 = (p_pred[1][1] * (p_pred[0][0] + r_p) - p_pred[0][1].powi(2)) / d;
        let k20 = (p_pred[0][2] * (p_pred[1][1] + r_v) - p_pred[1][2] * p_pred[0][1]) / d;
        let k21 = (p_pred[1][2] * (p_pred[0][0] + r_p) - p_pred[0][2] * p_pred[0][1]) / d;

        // 4. Update State
        let y_p = z_pos - pos_pred;
        let y_v = z_vel - vel_pred;

        let pos_new = pos_pred + k00 * y_p + k01 * y_v;
        let vel_new = vel_pred + k10 * y_p + k11 * y_v;
        let acc_new = acc_pred + k20 * y_p + k21 * y_v;

        // 5. Update Covariance: P_new = (I - K*H)*P_pred
        let mut p_new = [[0.0; 3]; 3];
        p_new[0][0] = (1.0 - k00) * p_pred[0][0] - k01 * p_pred[0][1];
        p_new[0][1] = (1.0 - k00) * p_pred[0][1] - k01 * p_pred[1][1];
        p_new[0][2] = (1.0 - k00) * p_pred[0][2] - k01 * p_pred[1][2];

        p_new[1][1] = -k10 * p_pred[0][1] + (1.0 - k11) * p_pred[1][1];
        p_new[1][2] = -k10 * p_pred[0][2] + (1.0 - k11) * p_pred[1][2];

        p_new[2][2] = -k20 * p_pred[0][2] - k21 * p_pred[1][2] + p_pred[2][2];

        // Enforce symmetry and positive semi-definiteness on diagonal
        p_new[1][0] = p_new[0][1];
        p_new[2][0] = p_new[0][2];
        p_new[2][1] = p_new[1][2];

        p_new[0][0] = p_new[0][0].max(0.0);
        p_new[1][1] = p_new[1][1].max(0.0);
        p_new[2][2] = p_new[2][2].max(0.0);

        (pos_new, vel_new, acc_new, p_new)
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum RadarState {
    Scanning,
    Tracking { contact_id: u32 },
    Jamming { contact_id: u32 },
}

#[derive(Clone, Copy, Debug)]
pub struct RadarJob {
    pub angle: f64,
    pub width: f64,
    pub min_distance: f64,
    pub max_distance: f64,
    pub state: RadarState,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ScanSlice {
    pub angle: f64,
    pub width: f64,
    pub min_distance: f64,
    pub max_distance: f64,
}

pub trait ScanSliceGenerator {
    fn next_slice(&mut self, target: Option<&Contact>) -> ScanSlice;
    fn notify_hit(&mut self) {}
    fn notify_non_missile_contact(&mut self, _has: bool) {}
}

pub struct DefaultScanSliceGenerator {
    pub base_search_width: f64,
    pub base_max_distance: f64,
    pub search_width: f64,
    pub max_distance: f64,
    pub last_scan_heading: Option<f64>,
    pub swept_angle: f64,
    pub hit_seen_in_cycle: bool,
    pub has_non_missile_contact: bool,
}

impl DefaultScanSliceGenerator {
    pub fn new(search_width: f64, max_distance: f64) -> Self {
        Self {
            base_search_width: search_width,
            base_max_distance: max_distance,
            search_width,
            max_distance,
            last_scan_heading: None,
            swept_angle: 0.0,
            hit_seen_in_cycle: false,
            has_non_missile_contact: false,
        }
    }
}

impl ScanSliceGenerator for DefaultScanSliceGenerator {
    fn notify_hit(&mut self) {
        self.hit_seen_in_cycle = true;
    }

    fn notify_non_missile_contact(&mut self, has: bool) {
        self.has_non_missile_contact = has;
    }

    fn next_slice(&mut self, _target: Option<&Contact>) -> ScanSlice {
        if self.has_non_missile_contact {
            self.search_width = self.base_search_width;
            self.max_distance = self.base_max_distance;
            self.swept_angle = 0.0;
            self.hit_seen_in_cycle = false;
        }

        let last_angle = self.last_scan_heading.unwrap_or_else(|| radar_heading());
        let sweep_head = last_angle + self.search_width;
        self.last_scan_heading = Some(sweep_head);
        
        self.swept_angle += self.search_width;

        if self.swept_angle >= TAU - 1e-5 {
            if !self.has_non_missile_contact {
                if !self.hit_seen_in_cycle {
                    self.max_distance = (self.max_distance * 2.0).min(100000.0);
                    self.search_width = (self.search_width / 2.0).max(0.005);
                    debug!(
                        "No hits seen in full 360 degree scan. Adjusting parameters for next cycle: max_distance = {}, search_width = {}",
                        self.max_distance, self.search_width
                    );
                }
            }
            self.swept_angle = 0.0;
            self.hit_seen_in_cycle = false;
        }

        ScanSlice {
            angle: sweep_head,
            width: self.search_width,
            min_distance: 0.0,
            max_distance: self.max_distance,
        }
    }
}

pub struct RadarController {
    contacts: Vec<Contact>,
    non_provisional_contacts: Vec<Contact>,
    next_contact_id: u32,
    radar_states: Vec<RadarState>,
    prev_slices: Vec<Option<ScanSlice>>,
    current_target_id: Option<u32>,
    tracking_width: f64,
    gate_radius: f64,
    pub full_scans: u32,
    pub priority_targets: Vec<u32>,
    pub priority_track_interval: u32,
    scan_ticks: u32,
    last_scan_heading: Option<f64>,
    pub slice_generator: Box<dyn ScanSliceGenerator>,
    pub jamming_mode: bool,
}

impl RadarController {
    pub fn new() -> Self {
        let search_width = 0.6;
        let tracking_width = 0.05;
        let max_distance = 10000.0;
        let gate_radius = 200.0;
        Self {
            contacts: Vec::new(),
            non_provisional_contacts: Vec::new(),
            next_contact_id: 1,
            radar_states: vec![RadarState::Scanning; 2],
            prev_slices: vec![None; 2],
            current_target_id: None,
            tracking_width,
            gate_radius,
            full_scans: 0,
            priority_targets: Vec::new(),
            priority_track_interval: 6,
            scan_ticks: 0,
            last_scan_heading: None,
            slice_generator: Box::new(DefaultScanSliceGenerator::new(search_width, max_distance)),
            jamming_mode: false,
        }
    }

    pub fn add_radio_ping(&mut self, telemetry: TargetTelemetry) -> u32 {
        let current_t = current_tick();
        let elapsed_ticks = (current_t as u8).wrapping_sub(telemetry.tick) as f64;
        let dt = elapsed_ticks * TICK_LENGTH;
        let projected_position = telemetry.position + telemetry.velocity * dt;
        let c = ScanResult {
            position: projected_position,
            velocity: telemetry.velocity,
            class: telemetry.class,
            rssi: telemetry.rssi as f64,
            snr: RADIO_PING_SNR,
        };
        self.process_scan_hit(c, None)
    }

    pub fn process_scan_hit(&mut self, c: ScanResult, slice_width: Option<f64>) -> u32 {
        let current_t = current_tick();
        let mut best_match: Option<&mut Contact> = None;
        let mut best_dist = f64::MAX;

        // Calculate scan's hit position 99.99% CI radius
        let error_factor = 10.0f64.powf(-c.snr / 10.0);
        let dist_to_radar = position().distance(c.position);
        let sigma_r = 10000.0 * error_factor;
        let sigma_theta = (10.0 * (TAU / 360.0)) * error_factor;
        let pos_unc = sigma_r.max(dist_to_radar * sigma_theta.min(radar_width() / 2.0));
        let scan_ci_radius = 3.89 * pos_unc;

        for contact in &mut self.contacts {
            if contact.class == c.class {
                let expected_pos = contact.current_position();
                let dist = expected_pos.distance(c.position);
                let stats = contact.class.default_stats();
                let mut max_acc = stats.max_forward_acceleration
                    .max(stats.max_backward_acceleration)
                    .max(stats.max_lateral_acceleration);
                if contact.class == Class::Fighter || contact.class == Class::Missile {
                    max_acc += 100.0;
                }
                let dt_sec = current_t.wrapping_sub(contact.last_scanned) as f64 * TICK_LENGTH;
                let fallback = 0.5 * max_acc * dt_sec * dt_sec;
                let gate_radius = (contact.ci_mult() * contact.current_pos_uncertainty()).max(10.0) + fallback;
                
                // Count match if scan is within contact's CI (gate_radius) OR contact is within scan's CI (scan_ci_radius)
                let is_match = dist < gate_radius || dist < scan_ci_radius;
                if is_match && dist < best_dist {
                    best_dist = dist;
                    best_match = Some(contact);
                }
            }
        }

        let returned_id = if let Some(contact) = best_match {
            let ci_radius = contact.ci_mult() * contact.current_pos_uncertainty();
            if best_dist > ci_radius.max(10.0) && best_dist > 20.0 {
                let stats = contact.class.default_stats();
                let mut max_acc = stats.max_forward_acceleration
                    .max(stats.max_backward_acceleration)
                    .max(stats.max_lateral_acceleration);
                if contact.class == Class::Fighter || contact.class == Class::Missile {
                    max_acc += 100.0;
                }
                let dt_sec = current_t.wrapping_sub(contact.last_scanned) as f64 * TICK_LENGTH;
                let fallback = 0.5 * max_acc * dt_sec * dt_sec;
                let gate_radius = ci_radius.max(10.0) + fallback;
                if best_dist > gate_radius {
                    debug!("Scan hit for contact {} was outside contact's CI and moved position by {:.1}m (>20m), associated because it is within scan's 99.99% CI ({:.1}m)", contact.id, best_dist, scan_ci_radius);
                } else {
                    debug!("Scan hit for contact {} was outside CI and moved position by {:.1}m (>20m), associated due to dynamic gating fallback ({:.1}m based on max accel {:.1}m/s^2 and dt={:.3}s)", contact.id, best_dist, fallback, max_acc, dt_sec);
                }
            }

            let error_factor = 10.0f64.powf(-c.snr / 10.0);
            let dist = position().distance(c.position);
            let sigma_r = 10000.0 * error_factor;
            let sigma_theta = (10.0 * (TAU / 360.0)) * error_factor;
            let pos_unc = sigma_r.max(dist * sigma_theta.min(radar_width() / 2.0));
            let vel_unc = 100.0 * error_factor;

            let prev_unc = contact.prev_scan_pos_uncertainty.unwrap_or(contact.pos_uncertainty);
            let predicted_unc = contact.current_pos_uncertainty();
            contact.predict_and_update(current_t, c.position, c.velocity, pos_unc, vel_unc);
            let current_unc = contact.pos_uncertainty;
            let pct_improvement = if predicted_unc > 0.0 {
                (1.0 - current_unc / predicted_unc) * 100.0
            } else {
                0.0
            };
            debug!(
                "Contact {} uncertainty improved by {:.1}% on subsequent tick (predicted CI: {:.2}m -> updated CI: {:.2}m)",
                contact.id, pct_improvement, contact.ci_mult() * predicted_unc, contact.ci_mult() * current_unc
            );
            if contact.prioritize_scan {
                let current_unc = contact.pos_uncertainty;
                let shrink_ratio = current_unc / prev_unc;
                let improvement_pct = (1.0 - shrink_ratio) * 100.0;
                if improvement_pct < 5.0 {
                    contact.low_improvement_consecutive_scans += 1;
                    debug!(
                        "Contact {} (Missile) CI went from {:.2} to {:.2}. Shrink was {:.1}% (<5.0% threshold). Low-improvement scans: {}.",
                        contact.id,
                        contact.ci_mult() * prev_unc,
                        contact.ci_mult() * current_unc,
                        improvement_pct,
                        contact.low_improvement_consecutive_scans
                    );
                } else {
                    contact.low_improvement_consecutive_scans = 0;
                    debug!(
                        "Contact {} (Missile) CI went from {:.2} to {:.2}. Shrink was {:.1}% (>=5.0%). Reset low-improvement counter.",
                        contact.id,
                        contact.ci_mult() * prev_unc,
                        contact.ci_mult() * current_unc,
                        improvement_pct
                    );
                }

                if contact.low_improvement_consecutive_scans >= 2 {
                    contact.prioritize_scan = false;
                    debug!(
                        "Contact {} (Missile) reached 2 consecutive low-improvement tracking scans. Stop prioritizing.",
                        contact.id
                    );
                } else {
                    debug!(
                        "Contact {} (Missile) is still improving (low-improvement count: {}). Keep prioritizing.",
                        contact.id,
                        contact.low_improvement_consecutive_scans
                    );
                }
            }
            contact.prev_scan_pos_uncertainty = Some(contact.pos_uncertainty);
            contact.rssi = c.rssi;
            contact.snr = c.snr;
            contact.radar_locked = true;
            contact.tracking_retry_count = 0;
            if c.snr == RADIO_PING_SNR {
                contact.provisional = false; // Confirm immediately if it is a radio ping
            }
            contact.id
        } else {
            let error_factor = 10.0f64.powf(-c.snr / 10.0);
            let dist = position().distance(c.position);
            let sigma_r = 10000.0 * error_factor;
            let sigma_theta = (10.0 * (TAU / 360.0)) * error_factor;
            let pos_unc = sigma_r.max(dist * sigma_theta.min(radar_width() / 2.0));
            let ci_radius = 3.89 * pos_unc;

            let preexisting = self.contacts.iter().find(|co| {
                co.class == c.class && co.current_position().distance(c.position) <= ci_radius
            });

            if let Some(existing) = preexisting {
                existing.id
            } else {
                if let Some(w) = slice_width {
                    if !is_within_reliable_range(c.class, dist, w) {
                        return 0;
                    }
                }
                let vel_unc = 100.0 * error_factor;
                let last_scanned = current_t;
                let cov_x = Contact::initial_cov(pos_unc, vel_unc, c.class);
                let cov_y = Contact::initial_cov(pos_unc, vel_unc, c.class);

                let new_id = self.next_contact_id;
                self.contacts.push(Contact {
                    id: new_id,
                    kinematic: KinematicState::new(c.class, c.position, c.velocity, Vec2::new(0.0, 0.0), last_scanned),
                    rssi: c.rssi,
                    snr: c.snr,
                    pos_uncertainty: pos_unc,
                    vel_uncertainty: vel_unc,
                    radar_locked: true,
                    provisional: c.snr != RADIO_PING_SNR, // Confirm immediately if it is a radio ping
                    tracking_retry_count: 0,
                    confirmation_attempts: 0,
                    unscanned_in_range_ticks: 0,
                    p_cov_x: cov_x,
                    p_cov_y: cov_y,
                    prioritize_scan: c.class == Class::Missile,
                    prev_scan_pos_uncertainty: if c.class == Class::Missile { Some(pos_unc) } else { None },
                    low_improvement_consecutive_scans: 0,
                });
                self.next_contact_id += 1;
                new_id
            }
        };

        // Immediately remove enemy missiles or torpedoes moving away from us
        self.contacts.retain(|contact| {
            if contact.class == Class::Missile || contact.class == Class::Torpedo {
                let r = contact.current_position() - position();
                let v_rel = contact.current_velocity() - velocity();
                r.dot(v_rel) <= 0.0
            } else {
                true
            }
        });

        returned_id
    }

    pub fn set_tracking_width(&mut self, width: f64) {
        self.tracking_width = width;
    }

    pub fn set_gate_radius(&mut self, radius: f64) {
        self.gate_radius = radius;
    }

    pub fn contacts(&self) -> &[Contact] {
        &self.non_provisional_contacts
    }

    pub fn has_contact(&self, id: u32) -> bool {
        self.contacts.iter().any(|c| c.id == id)
    }

    pub fn get_contact(&self, id: u32) -> Option<&Contact> {
        self.contacts.iter().find(|c| c.id == id)
    }

    pub fn geometric_center(&self, default_pos: Vec2) -> Vec2 {
        if self.non_provisional_contacts.is_empty() {
            default_pos
        } else {
            let sum = self.non_provisional_contacts.iter().fold(Vec2::new(0.0, 0.0), |acc, c| acc + c.current_position());
            sum / self.non_provisional_contacts.len() as f64
        }
    }

    pub fn update(&mut self) {
        let current_t = current_tick();
        let num_radars = if class() == Class::Cruiser { 2 } else { 1 };

        let mut hit_seen_this_tick = false;

        // 1. Process scan results from previous tick depending on radar_states
        let mut scan_results = Vec::new();
        let mut scan_returned_none = vec![false; num_radars];
        for i in 0..num_radars {
            select_radar(i);
            if let Some(r) = scan() {
                hit_seen_this_tick = true;
                // Draw 99.99% confidence interval circle for this radar hit
                let error_factor = 10.0f64.powf(-r.snr / 10.0);
                let dist = position().distance(r.position);
                let sigma_r = 10000.0 * error_factor;
                let sigma_theta = (10.0 * (TAU / 360.0)) * error_factor;
                let pos_unc = sigma_r.max(dist * sigma_theta.min(radar_width() / 2.0));
                let radius = 3.89 * pos_unc;
                draw_polygon(r.position, radius, 32, 0.0, rgb(255, 255, 0));

                scan_results.push((i, r));
            } else {
                scan_returned_none[i] = true;
                if let RadarState::Tracking { contact_id } = self.radar_states[i] {
                    if let Some(intended_contact) = self.contacts.iter_mut().find(|co| co.id == contact_id) {
                        intended_contact.tracking_retry_count += 1;
                        debug!("Radar {} tracking scan for contact {} returned None. Incrementing retry count to {}.", i, contact_id, intended_contact.tracking_retry_count);
                    }
                }
            }
        }

        for (i, c) in scan_results {
            select_radar(i);
            let state = self.radar_states[i];
            let slice_width = self.prev_slices[i].map(|s| s.width);
            match state {
                RadarState::Scanning | RadarState::Jamming { .. } => {
                    self.process_scan_hit(c, slice_width);
                }
                RadarState::Tracking { contact_id } => {
                    let mut best_match_id = None;
                    let mut best_dist = f64::MAX;

                    // Calculate scan's hit position 99.99% CI radius
                    let error_factor = 10.0f64.powf(-c.snr / 10.0);
                    let dist_to_radar = position().distance(c.position);
                    let sigma_r = 10000.0 * error_factor;
                    let sigma_theta = (10.0 * (TAU / 360.0)) * error_factor;
                    let pos_unc = sigma_r.max(dist_to_radar * sigma_theta.min(radar_width() / 2.0));
                    let scan_ci_radius = 3.89 * pos_unc;

                    for contact in &self.contacts {
                        if contact.class != c.class {
                            continue;
                        }
                        let expected_pos = contact.current_position();
                        let dist = expected_pos.distance(c.position);
                        let ci_radius = contact.ci_mult() * contact.current_pos_uncertainty();
                        let stats = contact.class.default_stats();
                        let mut max_acc = stats.max_forward_acceleration
                            .max(stats.max_backward_acceleration)
                            .max(stats.max_lateral_acceleration);
                        if contact.class == Class::Fighter || contact.class == Class::Missile {
                            max_acc += 100.0;
                        }
                        let dt_sec = current_t.wrapping_sub(contact.last_scanned) as f64 * TICK_LENGTH;
                        let fallback = 0.5 * max_acc * dt_sec * dt_sec;
                        let gate_radius = 1.5 * (ci_radius.max(10.0) + fallback);
                        
                        // Count match if scan is within contact's CI (gate_radius) OR contact is within scan's CI (1.5 * scan_ci_radius)
                        let is_match = dist < gate_radius || dist < 1.5 * scan_ci_radius;
                        if is_match && dist < best_dist {
                            best_dist = dist;
                            best_match_id = Some(contact.id);
                        }
                    }

                    if let Some(best_id) = best_match_id {
                        {
                            let contact = self.contacts.iter_mut().find(|co| co.id == best_id).unwrap();
                            let ci_radius = contact.ci_mult() * contact.current_pos_uncertainty();
                            let stats = contact.class.default_stats();
                            let mut max_acc = stats.max_forward_acceleration
                                .max(stats.max_backward_acceleration)
                                .max(stats.max_lateral_acceleration);
                            if contact.class == Class::Fighter || contact.class == Class::Missile {
                                max_acc += 100.0;
                            }
                            let dt_sec = current_t.wrapping_sub(contact.last_scanned) as f64 * TICK_LENGTH;
                            let fallback = 0.5 * max_acc * dt_sec * dt_sec;
                            let gate_radius = 1.5 * (ci_radius.max(10.0) + fallback);
 
                            if best_dist > ci_radius.max(10.0) && best_dist > 20.0 {
                                if best_dist > gate_radius {
                                    debug!("Tracked scan hit associated to contact {} was outside contact's CI and moved position by {:.1}m (>20m), associated because it is within scan's 99.99% CI ({:.1}m)", contact.id, best_dist, 1.5 * scan_ci_radius);
                                } else {
                                    debug!("Tracked scan hit associated to contact {} was outside CI and moved position by {:.1}m (>20m), associated due to dynamic gating fallback ({:.1}m based on max accel {:.1}m/s^2 and dt={:.3}s)", contact.id, best_dist, fallback, max_acc, dt_sec);
                                }
                            }

                            let error_factor = 10.0f64.powf(-c.snr / 10.0);
                            let dist = position().distance(c.position);
                            let sigma_r = 10000.0 * error_factor;
                            let sigma_theta = (10.0 * (TAU / 360.0)) * error_factor;
                            let pos_unc = sigma_r.max(dist * sigma_theta.min(radar_width() / 2.0));
                            let vel_unc = 100.0 * error_factor;

                            let prev_unc = contact.prev_scan_pos_uncertainty.unwrap_or(contact.pos_uncertainty);
                            let predicted_unc = contact.current_pos_uncertainty();
                            contact.predict_and_update(current_t, c.position, c.velocity, pos_unc, vel_unc);
                            let current_unc = contact.pos_uncertainty;
                            let pct_improvement = if predicted_unc > 0.0 {
                                (1.0 - current_unc / predicted_unc) * 100.0
                            } else {
                                0.0
                            };
                            debug!(
                                "Contact {} uncertainty improved by {:.1}% on subsequent tick (predicted CI: {:.2}m -> updated CI: {:.2}m)",
                                contact.id, pct_improvement, contact.ci_mult() * predicted_unc, contact.ci_mult() * current_unc
                            );
                            if contact.prioritize_scan {
                                let current_unc = contact.pos_uncertainty;
                                let shrink_ratio = current_unc / prev_unc;
                                let improvement_pct = (1.0 - shrink_ratio) * 100.0;
                                if improvement_pct < 5.0 {
                                    contact.low_improvement_consecutive_scans += 1;
                                    debug!(
                                        "Contact {} (Missile) CI went from {:.2} to {:.2}. Shrink was {:.1}% (<5.0% threshold). Low-improvement scans: {}.",
                                        contact.id,
                                        contact.ci_mult() * prev_unc,
                                        contact.ci_mult() * current_unc,
                                        improvement_pct,
                                        contact.low_improvement_consecutive_scans
                                    );
                                } else {
                                    contact.low_improvement_consecutive_scans = 0;
                                    debug!(
                                        "Contact {} (Missile) CI went from {:.2} to {:.2}. Shrink was {:.1}% (>=5.0%). Reset low-improvement counter.",
                                        contact.id,
                                        contact.ci_mult() * prev_unc,
                                        contact.ci_mult() * current_unc,
                                        improvement_pct
                                    );
                                }

                                if contact.low_improvement_consecutive_scans >= 2 {
                                    contact.prioritize_scan = false;
                                    debug!(
                                        "Contact {} (Missile) reached 2 consecutive low-improvement tracking scans. Stop prioritizing.",
                                        contact.id
                                    );
                                } else {
                                    debug!(
                                        "Contact {} (Missile) is still improving (low-improvement count: {}). Keep prioritizing.",
                                        contact.id,
                                        contact.low_improvement_consecutive_scans
                                    );
                                }
                            }
                            contact.prev_scan_pos_uncertainty = Some(contact.pos_uncertainty);
                            contact.rssi = c.rssi;
                            contact.snr = c.snr;
                            contact.radar_locked = true;
                            contact.tracking_retry_count = 0;
                        }

                        // Check if the new position is definitely distinct from all existing lower-numbered contacts.
                        let mut is_distinct = true;
                        for other in &self.contacts {
                            if other.id < best_id {
                                let d = other.current_position().distance(c.position);
                                if d <= 30.0 {
                                    is_distinct = false;
                                    break;
                                }
                            }
                        }

                        let mut drop_this_contact = false;
                        {
                            let contact = self.contacts.iter_mut().find(|co| co.id == best_id).unwrap();
                            if contact.provisional {
                                contact.confirmation_attempts += 1;
                                if is_distinct {
                                    contact.provisional = false;
                                    debug!("Confirmed contact {} because it is definitely distinct from lower-numbered contacts (>30m)", best_id);
                                } else {
                                    debug!("Contact {} remains unconfirmed (distance to a lower-numbered contact is <= 30m)", best_id);
                                }
                            }

                            if contact.provisional && contact.confirmation_attempts >= 3 {
                                debug!("Dropping contact {} because it could not be confirmed after 3 tracking attempts", best_id);
                                drop_this_contact = true;
                            }
                        }

                        if drop_this_contact {
                            self.contacts.retain(|other| other.id != best_id);
                        } else {
                            // Check duplicate dropping logic for best_id
                            let mut min_dist_lower_id: Option<f64> = None;
                            for other in &self.contacts {
                                if other.id < best_id {
                                    let d = other.current_position().distance(c.position);
                                    if min_dist_lower_id.is_none() || Some(d) < min_dist_lower_id {
                                        min_dist_lower_id = Some(d);
                                    }
                                }
                            }
                            if let Some(d) = min_dist_lower_id {
                                debug!("Tracked contact {} distance to closest lower ID contact: {:.1}m", best_id, d);
                            }

                            let mut should_drop_self = false;
                            self.contacts.retain(|other| {
                                if other.id == best_id {
                                    return true;
                                }
                                let d = other.current_position().distance(c.position);
                                if d < 15.0 {
                                    if other.id > best_id {
                                        debug!("Dropping duplicate contact {} (higher ID) at distance {:.1}m from tracked contact {}", other.id, d, best_id);
                                        return false;
                                    } else {
                                        should_drop_self = true;
                                    }
                                }
                                true
                            });

                            if should_drop_self {
                                debug!("Dropping tracked contact {} (higher ID) because it is within 15m of contact with lower ID", best_id);
                                self.contacts.retain(|other| other.id != best_id);
                            }
                        }
                    } else {
                        // Create a new contact
                        let error_factor = 10.0f64.powf(-c.snr / 10.0);
                        let dist = position().distance(c.position);

                        let mut should_add = true;
                        if let Some(w) = slice_width {
                            if !is_within_reliable_range(c.class, dist, w) {
                                should_add = false;
                            }
                        }

                        if should_add {
                            let sigma_r = 10000.0 * error_factor;
                            let sigma_theta = (10.0 * (TAU / 360.0)) * error_factor;
                            let pos_unc = sigma_r.max(dist * sigma_theta.min(radar_width() / 2.0));
                            let ci_radius = 3.89 * pos_unc;

                            let includes_preexisting = self.contacts.iter().any(|co| {
                                co.class == c.class && co.current_position().distance(c.position) <= ci_radius
                            });

                            if !includes_preexisting {
                                let vel_unc = 100.0 * error_factor;
                                let last_scanned = current_t;

                                let cov_x = Contact::initial_cov(pos_unc, vel_unc, c.class);
                                let cov_y = Contact::initial_cov(pos_unc, vel_unc, c.class);

                                self.contacts.push(Contact {
                                    id: self.next_contact_id,
                                    kinematic: KinematicState::new(c.class, c.position, c.velocity, Vec2::new(0.0, 0.0), last_scanned),
                                    rssi: c.rssi,
                                    snr: c.snr,
                                    pos_uncertainty: pos_unc,
                                    vel_uncertainty: vel_unc,
                                    radar_locked: true,
                                    provisional: true,
                                    tracking_retry_count: 0,
                                    confirmation_attempts: 0,
                                    unscanned_in_range_ticks: 0,
                                    p_cov_x: cov_x,
                                    p_cov_y: cov_y,
                                    prioritize_scan: c.class == Class::Missile,
                                    prev_scan_pos_uncertainty: if c.class == Class::Missile { Some(pos_unc) } else { None },
                                    low_improvement_consecutive_scans: 0,
                                });
                                self.next_contact_id += 1;
                            }
                        }
                    }

                    if best_match_id != Some(contact_id) {
                        debug!(
                            "Tracked contact {} was not matched by this scan (assigned to {:?}).",
                            contact_id, best_match_id
                        );
                        
                        // Draw red search gate for the missed contact
                        if let Some(contact) = self.contacts.iter().find(|co| co.id == contact_id) {
                            let expected_pos = contact.current_position();
                            let ci_radius = contact.ci_mult() * contact.current_pos_uncertainty();
                            let stats = contact.class.default_stats();
                            let mut max_acc = stats.max_forward_acceleration
                                .max(stats.max_backward_acceleration)
                                .max(stats.max_lateral_acceleration);
                            if contact.class == Class::Fighter || contact.class == Class::Missile {
                                max_acc += 100.0;
                            }
                            let dt_sec = current_t.wrapping_sub(contact.last_scanned) as f64 * TICK_LENGTH;
                            let fallback = 0.5 * max_acc * dt_sec * dt_sec;
                            let gate_radius = ci_radius.max(10.0) + fallback;
                            draw_polygon(expected_pos, gate_radius, 16, 0.0, rgb(255, 0, 0)); // Red color
                        }
                    }
                }
            }
        }

        // Update unscanned_in_range_ticks
        for contact in &mut self.contacts {
            if contact.last_scanned == current_t {
                contact.unscanned_in_range_ticks = 0;
            } else {
                let d = position().distance(contact.current_position());
                if is_within_range(contact, d) {
                    contact.unscanned_in_range_ticks += 1;
                }
            }
        }

        // Immediately remove enemy missiles or torpedoes moving away from us
        self.contacts.retain(|contact| {
            if contact.class == Class::Missile || contact.class == Class::Torpedo {
                let r = contact.current_position() - position();
                let v_rel = contact.current_velocity() - velocity();
                r.dot(v_rel) <= 0.0
            } else {
                true
            }
        });

        // 2.5. Prune tracked contacts that were not updated this tick by either physical radar or radio telemetry,
        // but only if they have failed tracking too many times (tracking_retry_count >= 3).
        self.contacts.retain(|contact| {
            let mut keep = true;
            if contact.tracking_retry_count >= 3 {
                if contact.last_scanned != current_t {
                    keep = false;
                    let next_pos = contact.position_at(current_t);
                    debug!("removed {}: pos=None expect=({:.1}, {:.1}) (failed tracking 3 times)", contact.id, next_pos.x, next_pos.y);
                }
            }
            keep
        });

        // 3. Prune old contacts (timeout after 120 ticks / 2 seconds of being in range and unscanned)
        self.contacts.retain(|c| {
            let keep = c.unscanned_in_range_ticks <= 120;
            if !keep {
                let current_pos = c.current_position();
                debug!("removed {}: pos=None expect=({:.1}, {:.1}) gate=None", c.id, current_pos.x, current_pos.y);
            }
            keep
        });

        // 4. Generate jobs for next tick
        let mut tracking_contacts = Vec::new();
        let mut tracking_jobs = Vec::new();

        if self.jamming_mode && !self.priority_targets.is_empty() {
            for contact in &self.contacts {
                if self.priority_targets.contains(&contact.id) {
                    let interval = self.priority_track_interval;
                    let next_track_tick = if contact.provisional {
                        current_t
                    } else {
                        contact.last_scanned.wrapping_add(interval * (1 + contact.tracking_retry_count))
                    };
                    tracking_contacts.push((true, next_track_tick, contact));
                }
            }
            tracking_contacts.sort_by_key(|&(_, next_track_tick, _)| next_track_tick);
            for (_, next_track_tick, contact) in tracking_contacts {
                if next_track_tick <= current_t {
                    if let Some(job) = self.generate_tracking_scan(contact, current_t) {
                        tracking_jobs.push(job);
                    }
                }
            }
        } else {
            for contact in &self.contacts {
                let is_priority = self.priority_targets.contains(&contact.id) || contact.provisional || contact.prioritize_scan;
                let interval = if is_priority { self.priority_track_interval } else { 20 };
                let next_track_tick = if contact.provisional {
                    current_t
                } else {
                    contact.last_scanned.wrapping_add(interval * (1 + contact.tracking_retry_count))
                };
                tracking_contacts.push((is_priority, next_track_tick, contact));
            }
            tracking_contacts.sort_by_key(|&(is_priority, next_track_tick, _)| (!is_priority, next_track_tick));
            for (_, next_track_tick, contact) in tracking_contacts {
                if next_track_tick <= current_t {
                    if let Some(job) = self.generate_tracking_scan(contact, current_t) {
                        tracking_jobs.push(job);
                    }
                }
            }
        }

        if hit_seen_this_tick {
            self.slice_generator.notify_hit();
        }
        let has_non_missile = self.contacts.iter().any(|c| c.class != Class::Missile);
        self.slice_generator.notify_non_missile_contact(has_non_missile);

        // Assign jobs to available radars
        let mut tracking_index = 0;
        for i in 0..num_radars {
            select_radar(i);
            set_radar_ecm_mode(EcmMode::None);

            if tracking_index < tracking_jobs.len() {
                let job = tracking_jobs[tracking_index];
                tracking_index += 1;

                set_radar_heading(job.angle);
                set_radar_width(job.width);
                set_radar_min_distance(job.min_distance);
                set_radar_max_distance(job.max_distance);
                
                self.radar_states[i] = job.state;
                self.prev_slices[i] = Some(ScanSlice {
                    angle: job.angle,
                    width: job.width,
                    min_distance: job.min_distance,
                    max_distance: job.max_distance,
                });
            } else if self.jamming_mode && !self.priority_targets.is_empty() {
                let high_priority_contacts: Vec<&Contact> = self.contacts.iter()
                    .filter(|c| self.priority_targets.contains(&c.id))
                    .collect();

                let mut jammed = false;
                if !high_priority_contacts.is_empty() {
                    let target_index = (current_t as usize + i) % high_priority_contacts.len();
                    let contact = high_priority_contacts[target_index];

                    if let Some(job) = self.generate_tracking_scan(contact, current_t) {
                        set_radar_heading(job.angle);
                        set_radar_width(job.width);
                        set_radar_min_distance(job.min_distance);
                        set_radar_max_distance(job.max_distance);
                        set_radar_ecm_mode(EcmMode::Noise);

                        self.radar_states[i] = RadarState::Jamming { contact_id: contact.id };
                        self.prev_slices[i] = Some(ScanSlice {
                            angle: job.angle,
                            width: job.width,
                            min_distance: job.min_distance,
                            max_distance: job.max_distance,
                        });
                        jammed = true;
                    }
                }

                if !jammed {
                    let target = self.current_target_id.and_then(|id| {
                        self.contacts.iter().find(|c| c.id == id)
                    });
                    let slice = self.slice_generator.next_slice(target);

                    set_radar_heading(slice.angle);
                    set_radar_width(slice.width);
                    set_radar_min_distance(slice.min_distance);
                    set_radar_max_distance(slice.max_distance);
                    
                    self.radar_states[i] = RadarState::Scanning;
                    self.prev_slices[i] = Some(slice);

                    self.last_scan_heading = Some(slice.angle);
                    self.scan_ticks += 1;
                    if self.scan_ticks >= 11 {
                        self.scan_ticks -= 11;
                        self.full_scans += 1;
                    }
                }
            } else {
                let target = self.current_target_id.and_then(|id| {
                    self.contacts.iter().find(|c| c.id == id)
                });
                let slice = self.slice_generator.next_slice(target);

                set_radar_heading(slice.angle);
                set_radar_width(slice.width);
                set_radar_min_distance(slice.min_distance);
                set_radar_max_distance(slice.max_distance);
                
                self.radar_states[i] = RadarState::Scanning;
                self.prev_slices[i] = Some(slice);

                self.last_scan_heading = Some(slice.angle);
                self.scan_ticks += 1;
                if self.scan_ticks >= 11 {
                    self.scan_ticks -= 11;
                    self.full_scans += 1;
                }
            }
        }

        // 5. Draw confidence intervals for all active contacts
        for contact in &self.contacts {
            let radius = contact.ci_mult() * contact.current_pos_uncertainty();
            draw_polygon(contact.current_position(), radius, 16, 0.0, rgb(255, 165, 0)); // Orange color

            // Draw maximum acceleration possible space
            let dt_sec = current_t.wrapping_sub(contact.last_scanned) as f64 * TICK_LENGTH;
            if dt_sec > 0.0 {
                let stats = contact.class.default_stats();
                let mut max_acc = stats.max_forward_acceleration
                    .max(stats.max_backward_acceleration)
                    .max(stats.max_lateral_acceleration);
                if contact.class == Class::Fighter || contact.class == Class::Missile {
                    max_acc += 100.0;
                }
                let displacement_factor = 0.5 * dt_sec * (dt_sec + TICK_LENGTH);
                let center = contact.current_position() - contact.acceleration * displacement_factor;
                let max_acc_radius = max_acc * displacement_factor;
                draw_polygon(center, max_acc_radius, 16, 0.0, rgb(0, 220, 255)); // Cyan/Light Blue color
            }
            
            // Draw a label with range and uncertainty
            let text_pos = contact.current_position() + vec2(0.0, radius + 20.0);
            draw_text!(text_pos, rgb(255, 165, 0), "cid: {}", contact.id);
        }

        // Cache non-provisional contacts
        self.non_provisional_contacts = self.contacts.iter()
            .filter(|c| !c.provisional)
            .cloned()
            .collect();
    }

    fn is_contact_matched_by_beam(&self, contact: &Contact, beam: &RadarJob, current_t: u32) -> bool {
        let next_our_pos = position() + velocity() * TICK_LENGTH;
        let next_contact_pos = contact.position_at(current_t + 1);
        let v = next_contact_pos - next_our_pos;
        let contact_d = v.length();
        let contact_angle = v.angle();

        let next_pos_uncertainty = contact.pos_uncertainty_at(current_t + 1);
        let gate_radius = (contact.ci_mult() * next_pos_uncertainty).max(self.gate_radius);
        let gate_angle = gate_radius / contact_d;

        let d_overlap = contact_d + gate_radius >= beam.min_distance && contact_d - gate_radius <= beam.max_distance;
        let a_diff = angle_diff(beam.angle, contact_angle).abs();
        let a_overlap = a_diff <= beam.width / 2.0 + gate_angle;

        d_overlap && a_overlap
    }

    fn generate_tracking_scan(&self, contact: &Contact, current_t: u32) -> Option<RadarJob> {
        let next_pos = contact.position_at(current_t + 1);
        let next_our_pos = position() + velocity() * TICK_LENGTH;
        let d = next_our_pos.distance(next_pos);
        if !is_within_range(contact, d) {
            return None;
        }
        let target_angle = (next_pos - next_our_pos).angle();

        let next_pos_uncertainty = contact.pos_uncertainty_at(current_t + 1);
        let gate_radius = (contact.ci_mult() * next_pos_uncertainty).max(self.gate_radius);

        let calculated_width = clamped_tracking_width(
            contact,
            d,
            gate_radius,
            next_pos_uncertainty,
            self.tracking_width,
        );
        let min_distance = (d - (contact.ci_mult() * next_pos_uncertainty).max(10.0)).max(0.0);
        let max_distance = d + (contact.ci_mult() * next_pos_uncertainty).max(10.0);

        let initial_job = RadarJob {
            angle: target_angle,
            width: calculated_width.clamp(0.005, self.tracking_width),
            min_distance,
            max_distance,
            state: RadarState::Tracking { contact_id: contact.id },
        };

        // Collect all the contacts besides the target
        let ci_width = 2.0 * contact.ci_mult() * next_pos_uncertainty;
        let min_beam_width_at_range = 0.005 * d;
        let nearby_threshold = ci_width.max(min_beam_width_at_range);

        let mut nearby_contacts = Vec::new();
        for other in &self.contacts {
            if other.id == contact.id {
                continue;
            }
            let next_other_pos = other.position_at(current_t + 1);
            let dist_to_target = next_other_pos.distance(next_pos);
            if dist_to_target <= nearby_threshold {
                nearby_contacts.push(other);
            }
        }

        // Check if any other contact is matched by the initial tracking beam
        let mut any_other_matched = false;
        for other in &nearby_contacts {
            if self.is_contact_matched_by_beam(other, &initial_job, current_t) {
                any_other_matched = true;
                break;
            }
        }

        // If no other contact is matched, just use the initial beam.
        if !any_other_matched {
            return Some(initial_job);
        }

        // We have confusing contacts matched by the initial beam. Let's find alternatives.
        let mut closest_left_contact: Option<&Contact> = None;
        let mut min_left_diff = f64::MAX;

        let mut closest_right_contact: Option<&Contact> = None;
        let mut min_right_diff = f64::MAX;

        let mut closest_behind_contact: Option<&Contact> = None;
        let mut min_behind_diff = f64::MAX;

        let mut closest_front_contact: Option<&Contact> = None;
        let mut min_front_diff = f64::MAX;

        for other in &nearby_contacts {
            let other_pos = other.position_at(current_t + 1);
            let other_d = next_our_pos.distance(other_pos);
            let other_angle = (other_pos - next_our_pos).angle();

            // Angular differences
            let diff = angle_diff(target_angle, other_angle);
            if diff > 0.0 {
                // To the left
                if diff < min_left_diff {
                    min_left_diff = diff;
                    closest_left_contact = Some(other);
                }
            } else if diff < 0.0 {
                // To the right
                let abs_diff = diff.abs();
                if abs_diff < min_right_diff {
                    min_right_diff = abs_diff;
                    closest_right_contact = Some(other);
                }
            }

            // Radial distance differences
            if other_d > d {
                // Behind the target
                let diff_d = other_d - d;
                if diff_d < min_behind_diff {
                    min_behind_diff = diff_d;
                    closest_behind_contact = Some(other);
                }
            } else if other_d < d {
                // In front of the target
                let diff_d = d - other_d;
                if diff_d < min_front_diff {
                    min_front_diff = diff_d;
                    closest_front_contact = Some(other);
                }
            }
        }

        let mut candidates = Vec::new();
        candidates.push(initial_job);

        // a. Left contact midpoint
        if let Some(left_contact) = closest_left_contact {
            let left_angle = (left_contact.position_at(current_t + 1) - next_our_pos).angle();
            let angle_diff_val = angle_diff(target_angle, left_angle);
            let mid_angle = target_angle + angle_diff_val / 2.0;
            let initial_right_limit = target_angle - initial_job.width / 2.0;
            let width_raw = angle_diff(initial_right_limit, mid_angle);
            let clamped_width = width_raw.clamp(0.005, self.tracking_width);
            let new_angle = mid_angle - clamped_width / 2.0;
            candidates.push(RadarJob {
                angle: new_angle,
                width: clamped_width,
                min_distance: initial_job.min_distance,
                max_distance: initial_job.max_distance,
                state: RadarState::Tracking { contact_id: contact.id },
            });
        }

        // b. Right contact midpoint
        if let Some(right_contact) = closest_right_contact {
            let right_angle = (right_contact.position_at(current_t + 1) - next_our_pos).angle();
            let angle_diff_val = angle_diff(target_angle, right_angle);
            let mid_angle = target_angle + angle_diff_val / 2.0;
            let initial_left_limit = target_angle + initial_job.width / 2.0;
            let width_raw = angle_diff(mid_angle, initial_left_limit);
            let clamped_width = width_raw.clamp(0.005, self.tracking_width);
            let new_angle = mid_angle + clamped_width / 2.0;
            candidates.push(RadarJob {
                angle: new_angle,
                width: clamped_width,
                min_distance: initial_job.min_distance,
                max_distance: initial_job.max_distance,
                state: RadarState::Tracking { contact_id: contact.id },
            });
        }

        // c. Behind contact midpoint
        if let Some(behind_contact) = closest_behind_contact {
            let behind_pos = behind_contact.position_at(current_t + 1);
            let behind_d = next_our_pos.distance(behind_pos);
            let mid_d = (d + behind_d) / 2.0;
            candidates.push(RadarJob {
                angle: initial_job.angle,
                width: initial_job.width,
                min_distance: initial_job.min_distance,
                max_distance: mid_d,
                state: RadarState::Tracking { contact_id: contact.id },
            });
        }

        // d. In front contact midpoint
        if let Some(front_contact) = closest_front_contact {
            let front_pos = front_contact.position_at(current_t + 1);
            let front_d = next_our_pos.distance(front_pos);
            let mid_d = (d + front_d) / 2.0;
            candidates.push(RadarJob {
                angle: initial_job.angle,
                width: initial_job.width,
                min_distance: mid_d,
                max_distance: initial_job.max_distance,
                state: RadarState::Tracking { contact_id: contact.id },
            });
        }

        let contacts_match_by_beam = |job: &RadarJob| {
            nearby_contacts.iter()
                .filter(|other| self.is_contact_matched_by_beam(other, job, current_t))
                .count()
        };

        let contact_score = |b: &RadarJob| (contacts_match_by_beam(b), b.width.to_bits(), (b.max_distance - b.min_distance).to_bits());
        candidates.iter().min_by_key(|&b| contact_score(b)).cloned()
    }

    pub fn update_target(&mut self, our_pos: Vec2, our_vel: Vec2) -> Option<Contact> {
        if let Some(target_id) = self.current_target_id {
            if !self.contacts.iter().any(|c| c.id == target_id && !c.provisional) {
                self.current_target_id = None;
            }
        }

        if self.current_target_id.is_none() {
            let mut closest_id = None;
            let mut min_future_dist = f64::MAX;
            for contact in &self.contacts {
                if contact.provisional {
                    continue;
                }
                let t_horizon = 2.0;
                let target_pos_f = contact.position_at(current_tick() + (t_horizon / TICK_LENGTH).round() as u32);
                let our_pos_f = our_pos + our_vel * t_horizon;
                let future_dist = target_pos_f.distance(our_pos_f);

                if future_dist < min_future_dist {
                    min_future_dist = future_dist;
                    closest_id = Some(contact.id);
                }
            }
            self.current_target_id = closest_id;
        }

        self.current_target_id.and_then(|id| self.contacts.iter().find(|c| c.id == id).cloned())
    }

}

#[cfg(test)]
mod radar_test;
