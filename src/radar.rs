use oort_api::prelude::*;

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
    let d_back = d + 3.89 * next_pos_uncertainty;
    let (power, rx_xs) = own_radar_properties();
    let rcs = target_rcs(contact.class);
    let reliable_rssi = 1e-12; // -90 dBm
    let range_limited_width = (power * rcs * rx_xs) / (std::f64::consts::TAU * reliable_rssi * d_back.powi(4));
    (2.0 * gate_radius / d).min(range_limited_width).clamp(0.005, tracking_width)
}


#[derive(Clone, Debug)]
pub struct Contact {
    pub id: u32,
    pub class: Class,
    // The following fields represent the last ground truth measurement from a scan:
    pub position: Vec2,
    pub velocity: Vec2,
    pub acceleration: Vec2,
    pub last_scanned: u32,
    pub rssi: f64,
    pub snr: f64,
    pub pos_uncertainty: f64,
    pub vel_uncertainty: f64,
    pub radar_locked: bool,
    pub provisional: bool,
    pub tracking_retry_count: u32,
    pub confirmation_attempts: u32,
    pub confusing_contact: Option<u32>,
    pub p_cov_x: [[f64; 3]; 3],
    pub p_cov_y: [[f64; 3]; 3],
}

impl Contact {
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
        let dt = (tick - self.last_scanned) as f64 * TICK_LENGTH;
        self.position + self.velocity * dt + 0.5 * self.acceleration * dt * (dt + TICK_LENGTH)
    }

    pub fn velocity_at(&self, tick: u32) -> Vec2 {
        let dt = (tick - self.last_scanned) as f64 * TICK_LENGTH;
        self.velocity + self.acceleration * dt
    }

    pub fn pos_uncertainty_at(&self, tick: u32) -> f64 {
        let dt = (tick - self.last_scanned) as f64 * TICK_LENGTH;
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
        let dt = (current_t - self.last_scanned) as f64 * TICK_LENGTH;
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
            next_contact_id: 0,
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
        }
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
                if let RadarState::Tracking { contact_id } = self.radar_states[i] {
                    if let Some(contact) = self.contacts.iter().find(|co| co.id == contact_id) {
                        let next_pos = contact.position_at(current_t + 1);
                        let next_pos_uncertainty = contact.pos_uncertainty_at(current_t + 1);
                        let gate_radius = (3.89 * next_pos_uncertainty).max(self.gate_radius);
                        debug!("removed {}: pos=None expect=({:.1}, {:.1}) gate={:.1}", contact_id, next_pos.x, next_pos.y, gate_radius);
                    }
                    self.contacts.retain(|co| co.id != contact_id);
                }
            }
        }

        for (i, c) in scan_results {
            select_radar(i);
            match self.radar_states[i] {
                RadarState::Scanning => {
                    let mut best_match: Option<&mut Contact> = None;
                    let mut best_dist = f64::MAX;

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
                            let dt_sec = (current_t - contact.last_scanned) as f64 * TICK_LENGTH;
                            let fallback = 0.5 * max_acc * dt_sec * dt_sec;
                            let gate_radius = (3.89 * contact.current_pos_uncertainty()).max(10.0) + fallback;
                            if dist < gate_radius && dist < best_dist {
                                best_dist = dist;
                                best_match = Some(contact);
                            }
                        }
                    }

                    if let Some(contact) = best_match {
                        let ci_radius = 3.89 * contact.current_pos_uncertainty();
                        if best_dist > ci_radius.max(10.0) && best_dist > 20.0 {
                            let stats = contact.class.default_stats();
                            let mut max_acc = stats.max_forward_acceleration
                                .max(stats.max_backward_acceleration)
                                .max(stats.max_lateral_acceleration);
                            if contact.class == Class::Fighter || contact.class == Class::Missile {
                                max_acc += 100.0;
                            }
                            let dt_sec = (current_t - contact.last_scanned) as f64 * TICK_LENGTH;
                            let fallback = 0.5 * max_acc * dt_sec * dt_sec;
                            debug!("Scan hit for contact {} was outside 99.99% CI and moved position by {:.1}m (>20m), associated due to dynamic gating fallback ({:.1}m based on max accel {:.1}m/s^2 and dt={:.3}s)", contact.id, best_dist, fallback, max_acc, dt_sec);
                        }

                        let error_factor = 10.0f64.powf(-c.snr / 10.0);
                        let dist = position().distance(c.position);
                        let sigma_r = 10000.0 * error_factor;
                        let sigma_theta = (10.0 * (TAU / 360.0)) * error_factor;
                        let pos_unc = sigma_r.max(dist * sigma_theta.min(radar_width() / 2.0));
                        let vel_unc = 100.0 * error_factor;

                        contact.predict_and_update(current_t, c.position, c.velocity, pos_unc, vel_unc);
                        contact.rssi = c.rssi;
                        contact.snr = c.snr;
                        contact.radar_locked = true;
                        contact.tracking_retry_count = 0;
                    } else {
                        let error_factor = 10.0f64.powf(-c.snr / 10.0);
                        let dist = position().distance(c.position);
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
                                class: c.class,
                                position: c.position,
                                velocity: c.velocity,
                                acceleration: Vec2::new(0.0, 0.0),
                                last_scanned,
                                rssi: c.rssi,
                                snr: c.snr,
                                pos_uncertainty: pos_unc,
                                vel_uncertainty: vel_unc,
                                radar_locked: true,
                                provisional: true,
                                tracking_retry_count: 0,
                                confirmation_attempts: 0,
                                confusing_contact: None,
                                p_cov_x: cov_x,
                                p_cov_y: cov_y,
                            });
                            self.next_contact_id += 1;
                        }
                    }
                }
                RadarState::Tracking { contact_id } => {
                    let mut best_match_id = None;
                    let mut best_dist = f64::MAX;

                    for contact in &self.contacts {
                        if contact.class == c.class {
                            let expected_pos = contact.current_position();
                            let dist = expected_pos.distance(c.position);
                            let ci_radius = 3.89 * contact.current_pos_uncertainty();
                            let stats = contact.class.default_stats();
                            let mut max_acc = stats.max_forward_acceleration
                                .max(stats.max_backward_acceleration)
                                .max(stats.max_lateral_acceleration);
                            if contact.class == Class::Fighter || contact.class == Class::Missile {
                                max_acc += 100.0;
                            }
                            let dt_sec = (current_t - contact.last_scanned) as f64 * TICK_LENGTH;
                            let fallback = 0.5 * max_acc * dt_sec * dt_sec;
                            let gate_radius = 1.5 * (ci_radius.max(10.0) + fallback);
                            if dist < gate_radius && dist < best_dist {
                                best_dist = dist;
                                best_match_id = Some(contact.id);
                            }
                        }
                    }

                    if let Some(best_id) = best_match_id {
                        {
                            let contact = self.contacts.iter_mut().find(|co| co.id == best_id).unwrap();
                            let ci_radius = 3.89 * contact.current_pos_uncertainty();
                            let stats = contact.class.default_stats();
                            let mut max_acc = stats.max_forward_acceleration
                                .max(stats.max_backward_acceleration)
                                .max(stats.max_lateral_acceleration);
                            if contact.class == Class::Fighter || contact.class == Class::Missile {
                                max_acc += 100.0;
                            }
                            let dt_sec = (current_t - contact.last_scanned) as f64 * TICK_LENGTH;
                            let fallback = 0.5 * max_acc * dt_sec * dt_sec;

                            if best_dist > ci_radius.max(10.0) && best_dist > 20.0 {
                                debug!("Tracked scan hit associated to contact {} was outside 99.99% CI and moved position by {:.1}m (>20m), associated due to dynamic gating fallback ({:.1}m based on max accel {:.1}m/s^2 and dt={:.3}s)", contact.id, best_dist, fallback, max_acc, dt_sec);
                            }

                            let error_factor = 10.0f64.powf(-c.snr / 10.0);
                            let dist = position().distance(c.position);
                            let sigma_r = 10000.0 * error_factor;
                            let sigma_theta = (10.0 * (TAU / 360.0)) * error_factor;
                            let pos_unc = sigma_r.max(dist * sigma_theta.min(radar_width() / 2.0));
                            let vel_unc = 100.0 * error_factor;

                            contact.predict_and_update(current_t, c.position, c.velocity, pos_unc, vel_unc);
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
                            
                            // Reset confusing_contact since we matched successfully
                            contact.confusing_contact = None;

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
                                class: c.class,
                                position: c.position,
                                velocity: c.velocity,
                                acceleration: Vec2::new(0.0, 0.0),
                                last_scanned,
                                rssi: c.rssi,
                                snr: c.snr,
                                pos_uncertainty: pos_unc,
                                vel_uncertainty: vel_unc,
                                radar_locked: true,
                                provisional: true,
                                tracking_retry_count: 0,
                                confirmation_attempts: 0,
                                confusing_contact: None,
                                p_cov_x: cov_x,
                                p_cov_y: cov_y,
                            });
                            self.next_contact_id += 1;
                        }
                    }

                    if best_match_id != Some(contact_id) {
                        if let Some(intended_contact) = self.contacts.iter_mut().find(|co| co.id == contact_id) {
                            intended_contact.tracking_retry_count += 1;
                            let confusing_id = best_match_id.unwrap_or(self.next_contact_id - 1);
                            intended_contact.confusing_contact = Some(confusing_id);
                            debug!(
                                "Tracked contact {} was not matched by this scan (assigned to confusing contact {}). Retrying tracking (retry count: {}).",
                                contact_id, confusing_id, intended_contact.tracking_retry_count
                            );
                        }
                        
                        // Draw red search gate for the missed contact
                        if let Some(contact) = self.contacts.iter().find(|co| co.id == contact_id) {
                            let expected_pos = contact.current_position();
                            let ci_radius = 3.89 * contact.current_pos_uncertainty();
                            let stats = contact.class.default_stats();
                            let mut max_acc = stats.max_forward_acceleration
                                .max(stats.max_backward_acceleration)
                                .max(stats.max_lateral_acceleration);
                            if contact.class == Class::Fighter || contact.class == Class::Missile {
                                max_acc += 100.0;
                            }
                            let dt_sec = (current_t - contact.last_scanned) as f64 * TICK_LENGTH;
                            let fallback = 0.5 * max_acc * dt_sec * dt_sec;
                            let gate_radius = ci_radius.max(10.0) + fallback;
                            draw_polygon(expected_pos, gate_radius, 16, 0.0, rgb(255, 0, 0)); // Red color
                        }
                    }
                }
            }
        }

        // 3. Prune old contacts (timeout after 120 ticks / 2 seconds)
        self.contacts.retain(|c| {
            let keep = current_t - c.last_scanned <= 120;
            if !keep {
                let current_pos = c.current_position();
                debug!("removed {}: pos=None expect=({:.1}, {:.1}) gate=None", c.id, current_pos.x, current_pos.y);
            }
            keep
        });

        // 4. Generate jobs for next tick
        let mut tracking_contacts = Vec::new();
        for contact in &self.contacts {
            let is_priority = self.priority_targets.contains(&contact.id) || contact.provisional;
            let interval = if is_priority { self.priority_track_interval } else { 30 };
            let next_track_tick = if contact.provisional {
                current_t
            } else {
                contact.last_scanned + interval * (1 + contact.tracking_retry_count)
            };
            tracking_contacts.push((is_priority, next_track_tick, contact));
        }
        tracking_contacts.sort_by_key(|&(is_priority, next_track_tick, _)| (!is_priority, next_track_tick));

        let mut tracking_jobs = Vec::new();
        for (_, next_track_tick, contact) in tracking_contacts {
            if next_track_tick <= current_t {
                let next_pos = contact.position_at(current_t + 1);
                let next_our_pos = position() + velocity() * TICK_LENGTH;
                let d = next_our_pos.distance(next_pos);
                let mut angle = (next_pos - next_our_pos).angle();

                let next_pos_uncertainty = contact.pos_uncertainty_at(current_t + 1);
                let gate_radius = (3.89 * next_pos_uncertainty).max(self.gate_radius);

                let mut calculated_width = clamped_tracking_width(
                    contact,
                    d,
                    gate_radius,
                    next_pos_uncertainty,
                    self.tracking_width,
                );
                let mut min_distance = (d - (3.89 * next_pos_uncertainty).max(10.0)).max(0.0);
                let mut max_distance = d + (3.89 * next_pos_uncertainty).max(10.0);

                // Exclude confusing contact if present and active
                if let Some(confusing_id) = contact.confusing_contact {
                    if let Some(confusing_contact) = self.contacts.iter().find(|c| c.id == confusing_id) {
                        let confusing_pos = confusing_contact.position_at(current_t + 1);
                        let confusing_d = next_our_pos.distance(confusing_pos);
                        let confusing_angle = (confusing_pos - next_our_pos).angle();
                        
                        let confusing_unc = confusing_contact.pos_uncertainty_at(current_t + 1);
                        let confusing_gate = (3.89 * confusing_unc).max(self.gate_radius);

                        // Try adjusting the filter distance first: scanning either in front of or behind confusing contact.
                        let mut distance_adjusted = false;
                        if d < confusing_d {
                            // Confusing contact is behind the target. Try to scan in front of it.
                            let limit = confusing_d - confusing_gate;
                            if limit > d - gate_radius {
                                max_distance = max_distance.min(limit);
                                if max_distance > min_distance {
                                    distance_adjusted = true;
                                    debug!("Excluding confusing contact {} for target {} by adjusting distance (scanning in front: max_distance = {:.1}m)", confusing_id, contact.id, max_distance);
                                }
                            }
                        } else if d > confusing_d {
                            // Confusing contact is in front of the target. Try to scan behind it.
                            let limit = confusing_d + confusing_gate;
                            if limit < d + gate_radius {
                                min_distance = min_distance.max(limit);
                                if max_distance > min_distance {
                                    distance_adjusted = true;
                                    debug!("Excluding confusing contact {} for target {} by adjusting distance (scanning behind: min_distance = {:.1}m)", confusing_id, contact.id, min_distance);
                                }
                            }
                        }

                        // If distance exclusion is not possible, adjust the radar sweep angle and width
                        if !distance_adjusted {
                            let gate_angle = gate_radius / d;
                            let confusing_gate_angle = confusing_gate / confusing_d;
                            
                            // Normalize confusing_angle relative to target angle to handle TAU wrap-around
                            let norm_confusing_angle = angle + angle_diff(angle, confusing_angle);
                            
                            if norm_confusing_angle > angle {
                                let confusing_min_a = norm_confusing_angle - confusing_gate_angle;
                                let target_min_a = angle - gate_angle;
                                let target_max_a = angle + gate_angle;
                                let new_max_a = target_max_a.min(confusing_min_a);
                                if new_max_a > target_min_a {
                                    angle = (target_min_a + new_max_a) / 2.0;
                                    calculated_width = new_max_a - target_min_a;
                                    debug!("Excluding confusing contact {} for target {} by shifting sweep to the right (angle = {:.3} rad, width = {:.3} rad)", confusing_id, contact.id, angle, calculated_width);
                                }
                            } else {
                                let confusing_max_a = norm_confusing_angle + confusing_gate_angle;
                                let target_min_a = angle - gate_angle;
                                let target_max_a = angle + gate_angle;
                                let new_min_a = target_min_a.max(confusing_max_a);
                                if target_max_a > new_min_a {
                                    angle = (new_min_a + target_max_a) / 2.0;
                                    calculated_width = target_max_a - new_min_a;
                                    debug!("Excluding confusing contact {} for target {} by shifting sweep to the left (angle = {:.3} rad, width = {:.3} rad)", confusing_id, contact.id, angle, calculated_width);
                                }
                            }
                        }
                    }
                }

                tracking_jobs.push(RadarJob {
                    angle,
                    width: calculated_width.clamp(0.005, self.tracking_width),
                    min_distance,
                    max_distance,
                    state: RadarState::Tracking { contact_id: contact.id },
                });
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
            if tracking_index < tracking_jobs.len() {
                let job = tracking_jobs[tracking_index];
                tracking_index += 1;

                select_radar(i);
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
            } else {
                let target = self.current_target_id.and_then(|id| {
                    self.contacts.iter().find(|c| c.id == id)
                });
                let slice = self.slice_generator.next_slice(target);

                select_radar(i);
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

        // 5. Draw 99.99% confidence intervals for all active contacts
        for contact in &self.contacts {
            let radius = 3.89 * contact.current_pos_uncertainty();
            draw_polygon(contact.current_position(), radius, 16, 0.0, rgb(255, 165, 0)); // Orange color

            // Draw maximum acceleration possible space
            let dt_sec = (current_t - contact.last_scanned) as f64 * TICK_LENGTH;
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

    /// Updates or inserts a contact received via radio telemetry,
    /// and configures the radar to track it.
    pub fn update_from_radio(&mut self, pos: Vec2, vel: Vec2, accel: Vec2, existing_id: Option<u32>) -> u32 {
        let current_t = current_tick();
        let mut found_id = None;

        // If an existing target ID is provided and is currently tracked, update it directly
        // to prevent target switching/swapping.
        if let Some(id) = existing_id {
            if let Some(contact) = self.contacts.iter_mut().find(|c| c.id == id) {
                contact.position = pos;
                contact.velocity = vel;
                contact.acceleration = accel;
                contact.last_scanned = current_t;
                contact.pos_uncertainty = 10.0;
                contact.vel_uncertainty = 5.0;
                contact.tracking_retry_count = 0;
                contact.p_cov_x = Contact::initial_cov(10.0, 5.0, contact.class);
                contact.p_cov_y = Contact::initial_cov(10.0, 5.0, contact.class);
                found_id = Some(id);
            }
        }

        if found_id.is_none() {
            for contact in &mut self.contacts {
                // Match based on a proximity gate of 500m
                if contact.position.distance(pos) < 500.0 {
                    contact.position = pos;
                    contact.velocity = vel;
                    contact.acceleration = accel;
                    contact.last_scanned = current_t;
                    // High precision radio telemetry resets uncertainty
                    contact.pos_uncertainty = 10.0;
                    contact.vel_uncertainty = 5.0;
                    contact.tracking_retry_count = 0;
                    contact.p_cov_x = Contact::initial_cov(10.0, 5.0, contact.class);
                    contact.p_cov_y = Contact::initial_cov(10.0, 5.0, contact.class);
                    found_id = Some(contact.id);
                    break;
                }
            }
        }

        let contact_id = if let Some(id) = found_id {
            id
        } else if let Some(id) = existing_id {
            // Recreate deleted contact using its existing ID
            let last_scanned = current_t;
            let cov_x = Contact::initial_cov(10.0, 5.0, Class::Fighter);
            let cov_y = Contact::initial_cov(10.0, 5.0, Class::Fighter);
            self.contacts.push(Contact {
                id,
                class: Class::Fighter,
                position: pos,
                velocity: vel,
                acceleration: accel,
                last_scanned,
                rssi: 0.0,
                snr: 50.0,
                pos_uncertainty: 10.0,
                vel_uncertainty: 5.0,
                radar_locked: false,
                provisional: false,
                tracking_retry_count: 0,
                confirmation_attempts: 0,
                confusing_contact: None,
                p_cov_x: cov_x,
                p_cov_y: cov_y,
            });
            id
        } else {
            let id = self.next_contact_id;
            let last_scanned = current_t;
            let cov_x = Contact::initial_cov(10.0, 5.0, Class::Fighter);
            let cov_y = Contact::initial_cov(10.0, 5.0, Class::Fighter);
            self.contacts.push(Contact {
                id,
                class: Class::Fighter,
                position: pos,
                velocity: vel,
                acceleration: accel,
                last_scanned,
                rssi: 0.0,
                snr: 50.0,
                pos_uncertainty: 10.0,
                vel_uncertainty: 5.0,
                radar_locked: false,
                provisional: false,
                tracking_retry_count: 0,
                confirmation_attempts: 0,
                confusing_contact: None,
                p_cov_x: cov_x,
                p_cov_y: cov_y,
            });
            self.next_contact_id += 1;
            id
        };

        // Force radar to track this contact and configure the hardware immediately
        self.radar_states[0] = RadarState::Tracking { contact_id };
        select_radar(0);
        if let Some(contact) = self.contacts.iter().find(|c| c.id == contact_id) {
            let next_pos = contact.position_at(current_t + 1);
            let next_our_pos = position() + velocity() * TICK_LENGTH;
            let d = next_our_pos.distance(next_pos);
            let angle = (next_pos - next_our_pos).angle();
            set_radar_heading(angle);
            
            let next_pos_uncertainty = contact.pos_uncertainty_at(current_t + 1);
            let gate_radius = (3.89 * next_pos_uncertainty).max(self.gate_radius);
            
            let calculated_width = clamped_tracking_width(
                contact,
                d,
                gate_radius,
                next_pos_uncertainty,
                self.tracking_width,
            );
            set_radar_width(calculated_width);
            
            let ci_radius = (3.89 * next_pos_uncertainty).max(10.0);
            set_radar_min_distance((d - ci_radius).max(0.0));
            set_radar_max_distance(d + ci_radius);
        }

        self.non_provisional_contacts = self.contacts.iter()
            .filter(|c| !c.provisional)
            .cloned()
            .collect();

        contact_id
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_kalman_filter() {
        let mut contact = Contact {
            id: 0,
            class: Class::Fighter,
            position: Vec2::new(0.0, 0.0),
            velocity: Vec2::new(10.0, 0.0),
            acceleration: Vec2::new(0.0, 0.0),
            last_scanned: 0,
            rssi: 0.0,
            snr: 30.0,
            pos_uncertainty: 20.0,
            vel_uncertainty: 10.0,
            radar_locked: true,
            provisional: false,
            tracking_retry_count: 0,
            confirmation_attempts: 0,
            confusing_contact: None,
            p_cov_x: Contact::initial_cov(20.0, 10.0, Class::Fighter),
            p_cov_y: Contact::initial_cov(20.0, 10.0, Class::Fighter),
        };

        // Check initial covariance properties
        assert!(contact.p_cov_x[0][0] > 0.0);
        assert!(contact.p_cov_x[1][1] > 0.0);
        assert!(contact.p_cov_x[2][2] > 0.0);
        assert_eq!(contact.p_cov_x[0][1], 0.0);

        // Run a simulation of 10 ticks (approx 0.16 seconds)
        let mut t = 0;
        let sigma_p = 10.0;
        let sigma_v = 5.0;

        for _ in 0..10 {
            t += 1;
            let dt = TICK_LENGTH;
            let true_pos = Vec2::new(10.0 * (t as f64) * dt, 0.0);
            let true_vel = Vec2::new(10.0, 0.0);

            // Add simple alternating noise to make it noisy but zero-mean
            let noise_sign = if t % 2 == 0 { 1.0 } else { -1.0 };
            let meas_pos = true_pos + Vec2::new(noise_sign * sigma_p, 0.0);
            let meas_vel = true_vel + Vec2::new(-noise_sign * sigma_v, 0.0);

            contact.predict_and_update(t, meas_pos, meas_vel, sigma_p, sigma_v);

            // Check that matrix symmetry is preserved
            assert!((contact.p_cov_x[0][1] - contact.p_cov_x[1][0]).abs() < 1e-9);
            assert!((contact.p_cov_x[0][2] - contact.p_cov_x[2][0]).abs() < 1e-9);
            assert!((contact.p_cov_x[1][2] - contact.p_cov_x[2][1]).abs() < 1e-9);
            assert!((contact.p_cov_y[0][1] - contact.p_cov_y[1][0]).abs() < 1e-9);
            assert!((contact.p_cov_y[0][2] - contact.p_cov_y[2][0]).abs() < 1e-9);
            assert!((contact.p_cov_y[1][2] - contact.p_cov_y[2][1]).abs() < 1e-9);

            // Diagonals must remain positive
            assert!(contact.p_cov_x[0][0] >= 0.0);
            assert!(contact.p_cov_x[1][1] >= 0.0);
            assert!(contact.p_cov_x[2][2] >= 0.0);
        }

        // Verify that the final uncertainty is smaller than the initial, showing integration
        let final_pos_unc = contact.pos_uncertainty_at(t);
        assert!(final_pos_unc < 20.0, "Position uncertainty did not decrease! final={}", final_pos_unc);
    }

    #[test]
    fn test_radar_clamped_tracking_width() {
        let contact = Contact {
            id: 0,
            class: Class::Fighter,
            position: Vec2::new(0.0, 0.0),
            velocity: Vec2::new(0.0, 0.0),
            acceleration: Vec2::new(0.0, 0.0),
            last_scanned: 0,
            rssi: 0.0,
            snr: 30.0,
            pos_uncertainty: 100.0,
            vel_uncertainty: 10.0,
            radar_locked: true,
            provisional: false,
            tracking_retry_count: 0,
            confirmation_attempts: 0,
            confusing_contact: None,
            p_cov_x: Contact::initial_cov(100.0, 10.0, Class::Fighter),
            p_cov_y: Contact::initial_cov(100.0, 10.0, Class::Fighter),
        };

        let next_pos_uncertainty = 100.0f64;
        let gate_radius = (3.89 * next_pos_uncertainty).max(200.0);

        // At close range, geometric width tracking_width limit is active
        let w_close = clamped_tracking_width(&contact, 1000.0, gate_radius, next_pos_uncertainty, 0.05);
        assert_eq!(w_close, 0.05);

        // At very far range (100km), the range-limited width clamps it below 0.05
        let w_far = clamped_tracking_width(&contact, 100000.0, gate_radius, next_pos_uncertainty, 0.05);
        assert!(w_far < 0.05);
        assert!(w_far >= 0.005);
        assert!(w_far < w_close);
    }
}


