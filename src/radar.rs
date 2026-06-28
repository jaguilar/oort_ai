use crate::missile::TargetTelemetry;
use crate::physics::KinematicState;
use oort_api::prelude::*;

pub const RADIO_PING_SNR: f64 = 25.0;

pub fn ci_multiplier(confidence: f64) -> f64 {
    let p = 1.0 - (1.0 - confidence) / 2.0;
    let t = (-2.0 * (1.0 - p).max(1e-15).ln()).sqrt();
    let c = [2.515517, 0.802853, 0.010328];
    let d = [1.432788, 0.189269, 0.001308];
    let numerator = c[0] + c[1] * t + c[2] * t * t;
    let denominator = 1.0 + d[0] * t + d[1] * t * t + d[2] * t * t * t;
    t - numerator / denominator
}

// Returns the radar cross section of a ship of a given class.
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
    let range_limited_width =
        (power * rcs * rx_xs) / (std::f64::consts::TAU * reliable_rssi * d_back.powi(4));
    (2.0 * gate_radius / d)
        .min(range_limited_width)
        .clamp(0.005, tracking_width)
}

fn measurement_uncertainty(c: &ScanResult) -> (f64, f64) {
    let error_factor = 10.0f64.powf(-c.snr / 10.0);
    let dist = position().distance(c.position);
    let sigma_r = 10000.0 * error_factor;
    let sigma_theta = (10.0 * (TAU / 360.0)) * error_factor;
    let pos_unc = sigma_r.max(dist * sigma_theta.min(radar_width() / 2.0));
    let vel_unc = 100.0 * error_factor;
    (pos_unc, vel_unc)
}

fn is_within_range(contact: &Contact, d: f64) -> bool {
    let (power, rx_xs) = own_radar_properties();
    let rcs = target_rcs(contact.class);
    let reliable_rssi = 1e-12; // -90 dBm
    let max_range =
        ((power * rcs * rx_xs) / (0.005 * std::f64::consts::TAU * reliable_rssi)).powf(0.25);
    d <= max_range
}

fn is_within_reliable_range(class: Class, d: f64, slice_width: f64) -> bool {
    let (power, rx_xs) = own_radar_properties();
    let rcs = target_rcs(class);
    let reliable_rssi = 1e-12; // -90 dBm
    let max_range =
        ((power * rcs * rx_xs) / (slice_width * std::f64::consts::TAU * reliable_rssi)).powf(0.25);
    d <= max_range
}

#[derive(Clone, Debug)]
pub struct Contact {
    pub id: u32,
    pub kinematic: KinematicState,
    pub last_measurement_tick: u32,
    pub pos_uncertainty: f64,
    pub vel_uncertainty: f64,
    pub provisional: bool,
    pub tracking_retry_count: u32,
    pub confirmation_attempts: u32,
    pub unscanned_in_range_ticks: u32,
    pub p_cov_x: [[f64; 3]; 3],
    pub p_cov_y: [[f64; 3]; 3],
    pub prioritize_scan: bool,
    pub prev_scan_pos_uncertainty: Option<f64>,
    pub low_improvement_consecutive_scans: u32,
    pub last_beam_width: Option<f64>,
    pub last_beam_center: Option<f64>,
    pub last_beam_center_pos: Option<Vec2>,
    pub missile_scan_ticks_remaining: u32,
    pub scan_boundary_points: Option<[Vec2; 4]>,
    pub scan_boundary_vels: Option<[Vec2; 4]>,
    pub discovery_beam_width: f64,
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

fn normalize_angle(angle: f64) -> f64 {
    let mut a = angle % std::f64::consts::TAU;
    if a > std::f64::consts::PI {
        a -= std::f64::consts::TAU;
    } else if a < -std::f64::consts::PI {
        a += std::f64::consts::TAU;
    }
    a
}

impl Contact {
    pub fn ci_mult(&self) -> f64 {
        ci_multiplier(0.995)
    }

    pub fn initial_cov(pos_unc: f64, vel_unc: f64, class: Class) -> [[f64; 3]; 3] {
        let stats = class.default_stats();
        let mut max_acc = stats
            .max_forward_acceleration
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
        let mut max_acc = stats
            .max_forward_acceleration
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

    pub fn predict(&mut self, current_t: u32) {
        let dt = current_t.wrapping_sub(self.kinematic.last_scanned) as f64 * TICK_LENGTH;
        if dt <= 0.0 {
            return;
        }

        // Get process noise jerk spectral density S
        let stats = self.class.default_stats();
        let mut max_acc = stats
            .max_forward_acceleration
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

        self.p_cov_x = self.predict_cov_dim(self.p_cov_x, dt, f02, q00, q01, q02, q11, q12, q22);
        self.p_cov_y = self.predict_cov_dim(self.p_cov_y, dt, f02, q00, q01, q02, q11, q12, q22);

        self.kinematic.predict(current_t);

        self.pos_uncertainty = self.p_cov_x[0][0].sqrt().max(self.p_cov_y[0][0].sqrt());
        self.vel_uncertainty = self.p_cov_x[1][1].sqrt().max(self.p_cov_y[1][1].sqrt());
        self.predict_scan_boundary(TICK_LENGTH);
    }

    pub fn update_scan_boundary(&mut self, slice: ScanSlice) {
        let est_pos = self.current_position();
        let own_pos = position();
        let to_contact = est_pos - own_pos;
        let dist = to_contact.length().max(1e-6);
        let alpha = to_contact.y.atan2(to_contact.x);

        let ci_radius = self.ci_mult() * self.current_pos_uncertainty();
        let phi = (ci_radius / dist).min(1.0).asin();

        let rel_left_tangent = normalize_angle(alpha + phi - slice.angle);
        let rel_right_tangent = normalize_angle(alpha - phi - slice.angle);

        let cone_left = slice.width / 2.0;
        let left_angle_rel = if rel_left_tangent.abs() < cone_left {
            rel_left_tangent
        } else {
            cone_left
        };

        let cone_right = -slice.width / 2.0;
        let right_angle_rel = if rel_right_tangent.abs() < cone_right.abs() {
            rel_right_tangent
        } else {
            cone_right
        };

        let angle_left = slice.angle + left_angle_rel;
        let angle_right = slice.angle + right_angle_rel;

        let max_d = (dist + ci_radius).min(slice.max_distance);
        let min_d = (dist - ci_radius)
            .max(slice.min_distance)
            .max(50.0)
            .min(max_d);

        let c1 = own_pos + vec2(angle_left.cos(), angle_left.sin()) * min_d;
        let c2 = own_pos + vec2(angle_left.cos(), angle_left.sin()) * max_d;
        let c3 = own_pos + vec2(angle_right.cos(), angle_right.sin()) * min_d;
        let c4 = own_pos + vec2(angle_right.cos(), angle_right.sin()) * max_d;

        let center = (c1 + c2 + c3 + c4) / 4.0;
        let est_vel = self.current_velocity();
        let vel_ci_radius = self.ci_mult() * self.vel_uncertainty;
        let scale = vel_ci_radius * 2.0f64.sqrt();

        let get_vel = |c: Vec2| {
            let diff = c - center;
            let dir = if diff.length() > 1e-6 {
                diff.normalize()
            } else {
                Vec2::new(0.0, 0.0)
            };
            est_vel + dir * scale
        };

        let v1 = get_vel(c1);
        let v2 = get_vel(c2);
        let v3 = get_vel(c3);
        let v4 = get_vel(c4);

        self.scan_boundary_points = Some([c1, c2, c3, c4]);
        self.scan_boundary_vels = Some([v1, v2, v3, v4]);

        // // 1. Draw uncertainty circle at the time of scan
        // draw_polygon(
        //     est_pos,
        //     ci_radius,
        //     32,
        //     0.0,
        //     rgb(255, 255, 0), // Yellow color for the scan-time uncertainty circle
        // );
        // draw_text!(
        //     est_pos + vec2(0.0, -ci_radius - 15.0),
        //     rgb(255, 255, 0),
        //     "CI (cid {})",
        //     self.id
        // );

        // // 2. Draw tangent lines from own ship to the uncertainty circle at the time of scan
        // let d_sq = dist * dist - ci_radius * ci_radius;
        // let d_tangent = if d_sq > 0.0 { d_sq.sqrt() } else { dist };

        // let tang_l_dir = vec2((alpha + phi).cos(), (alpha + phi).sin());
        // let tang_r_dir = vec2((alpha - phi).cos(), (alpha - phi).sin());

        // let tang_l = own_pos + tang_l_dir * d_tangent;
        // let tang_r = own_pos + tang_r_dir * d_tangent;

        // draw_line(own_pos, tang_l, rgb(255, 128, 0)); // Orange for tangents
        // draw_line(own_pos, tang_r, rgb(255, 128, 0));

        // draw_text!(tang_l, rgb(255, 128, 0), "tang_L");
        // draw_text!(tang_r, rgb(255, 128, 0), "tang_R");

        // // 3. Draw the radar slice (beam cone) that got the hit
        // let slice_l_dir = vec2(
        //     (slice.angle + slice.width / 2.0).cos(),
        //     (slice.angle + slice.width / 2.0).sin(),
        // );
        // let slice_r_dir = vec2(
        //     (slice.angle - slice.width / 2.0).cos(),
        //     (slice.angle - slice.width / 2.0).sin(),
        // );

        // let slice_l_min = own_pos + slice_l_dir * slice.min_distance;
        // let slice_l_max = own_pos + slice_l_dir * slice.max_distance;
        // let slice_r_min = own_pos + slice_r_dir * slice.min_distance;
        // let slice_r_max = own_pos + slice_r_dir * slice.max_distance;

        // // Draw slice border lines in green
        // draw_line(slice_l_min, slice_l_max, rgb(0, 180, 0));
        // draw_line(slice_r_min, slice_r_max, rgb(0, 180, 0));
        // draw_line(slice_l_min, slice_r_min, rgb(0, 180, 0));
        // draw_line(slice_l_max, slice_r_max, rgb(0, 180, 0));

        // draw_text!(slice_l_max, rgb(0, 180, 0), "slice_L");
        // draw_text!(slice_r_max, rgb(0, 180, 0), "slice_R");

        // // 4. Draw the original computed boundary box (before prediction)
        // // Draw computed boundary box in pink/magenta
        // draw_line(c1, c2, rgb(255, 0, 255));
        // draw_line(c3, c4, rgb(255, 0, 255));
        // draw_line(c1, c3, rgb(255, 0, 255));
        // draw_line(c2, c4, rgb(255, 0, 255));

        // draw_text!(c2, rgb(255, 0, 255), "calc_L");
        // draw_text!(c4, rgb(255, 0, 255), "calc_R");
    }

    pub fn predict_scan_boundary(&mut self, dt: f64) {
        let class = self.class;
        let Some(ref mut points) = self.scan_boundary_points else {
            return;
        };
        let Some(ref mut vels) = self.scan_boundary_vels else {
            return;
        };

        let center = (points[0] + points[1] + points[2] + points[3]) / 4.0;

        let stats = class.default_stats();
        let mut max_acc = stats
            .max_forward_acceleration
            .max(stats.max_backward_acceleration)
            .max(stats.max_lateral_acceleration);
        if class == Class::Fighter || class == Class::Missile {
            max_acc += 100.0;
        }

        for i in 0..4 {
            let diff = points[i] - center;
            let dir = if diff.length() > 1e-6 {
                diff.normalize()
            } else {
                Vec2::new(0.0, 0.0)
            };
            vels[i] += dir * (max_acc * 2.0f64.sqrt() * dt);
            points[i] += vels[i] * dt;
        }
    }

    pub fn expand_scan_boundary(&mut self, frac: f64) {
        let Some(ref mut points) = self.scan_boundary_points else {
            return;
        };
        let center = (points[0] + points[1] + points[2] + points[3]) / 4.0;
        for i in 0..4 {
            let diff = points[i] - center;
            points[i] = center + diff * (1.0 + frac);
        }
    }

    fn predict_cov_dim(
        &self,
        p: [[f64; 3]; 3],
        dt: f64,
        f02: f64,
        q00: f64,
        q01: f64,
        q02: f64,
        q11: f64,
        q12: f64,
        q22: f64,
    ) -> [[f64; 3]; 3] {
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

        p_pred[1][0] = p_pred[0][1];
        p_pred[2][0] = p_pred[0][2];
        p_pred[2][1] = p_pred[1][2];

        p_pred
    }

    pub fn update_with_measurement(
        &mut self,
        z_pos: Vec2,
        z_vel: Vec2,
        sigma_p: f64,
        sigma_v: f64,
        current_t: u32,
    ) {
        let (pos_x, vel_x, acc_x, cov_x) = self.update_dim(
            self.position.x,
            self.velocity.x,
            self.acceleration.x,
            self.p_cov_x,
            z_pos.x,
            z_vel.x,
            sigma_p,
            sigma_v,
        );

        let (pos_y, vel_y, acc_y, cov_y) = self.update_dim(
            self.position.y,
            self.velocity.y,
            self.acceleration.y,
            self.p_cov_y,
            z_pos.y,
            z_vel.y,
            sigma_p,
            sigma_v,
        );

        self.position = Vec2::new(pos_x, pos_y);
        self.velocity = Vec2::new(vel_x, vel_y);
        self.acceleration = Vec2::new(acc_x, acc_y);
        self.p_cov_x = cov_x;
        self.p_cov_y = cov_y;
        self.last_measurement_tick = current_t;
        self.pos_uncertainty = self.p_cov_x[0][0].sqrt().max(self.p_cov_y[0][0].sqrt());
        self.vel_uncertainty = self.p_cov_x[1][1].sqrt().max(self.p_cov_y[1][1].sqrt());
    }

    fn update_dim(
        &self,
        pos_pred: f64,
        vel_pred: f64,
        acc_pred: f64,
        p_pred: [[f64; 3]; 3],
        z_pos: f64,
        z_vel: f64,
        sigma_p: f64,
        sigma_v: f64,
    ) -> (f64, f64, f64, [[f64; 3]; 3]) {
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

        let y_p = z_pos - pos_pred;
        let y_v = z_vel - vel_pred;

        let pos_new = pos_pred + k00 * y_p + k01 * y_v;
        let vel_new = vel_pred + k10 * y_p + k11 * y_v;
        let acc_new = acc_pred + k20 * y_p + k21 * y_v;

        let mut p_new = [[0.0; 3]; 3];
        p_new[0][0] = (1.0 - k00) * p_pred[0][0] - k01 * p_pred[0][1];
        p_new[0][1] = (1.0 - k00) * p_pred[0][1] - k01 * p_pred[1][1];
        p_new[0][2] = (1.0 - k00) * p_pred[0][2] - k01 * p_pred[1][2];

        p_new[1][1] = -k10 * p_pred[0][1] + (1.0 - k11) * p_pred[1][1];
        p_new[1][2] = -k10 * p_pred[0][2] + (1.0 - k11) * p_pred[1][2];

        p_new[2][2] = -k20 * p_pred[0][2] - k21 * p_pred[1][2] + p_pred[2][2];

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
    fn next_slice(&mut self, contacts: &[Contact]) -> ScanSlice;
    fn notify_hit(&mut self) {}
    fn notify_non_missile_contact(&mut self, _has: bool) {}
    fn search_width(&self) -> f64 {
        0.6
    }
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
    pub avoided_contact_id: Option<u32>,
    pub current_slice: Option<ScanSlice>,
    pub target_pos: Option<std::rc::Rc<std::cell::RefCell<Option<Vec2>>>>,
    pub biased_scan_width: f64,
    pub slice_index: usize,
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
            avoided_contact_id: None,
            current_slice: None,
            target_pos: None,
            biased_scan_width: 0.0,
            slice_index: 0,
        }
    }

    fn get_avoided_slice(&mut self, avoided_id: u32, contacts: &[Contact]) -> Option<ScanSlice> {
        let contact = contacts.iter().find(|c| c.id == avoided_id)?;
        let mut saved_slice = self.current_slice?;
        let proj_pts = project_boundary_points(contact)?;
        let next_our_pos = position() + velocity() * TICK_LENGTH;

        let min_d = if let Some(pt) = find_furthest_polygon_point_in_beam(
            proj_pts,
            saved_slice.angle,
            saved_slice.width,
            saved_slice.min_distance,
            saved_slice.max_distance,
            next_our_pos,
        ) {
            next_our_pos.distance(pt)
        } else {
            let mut max_dist = 0.0f64;
            for pt in &proj_pts {
                max_dist = max_dist.max(next_our_pos.distance(*pt));
            }
            max_dist
        };

        saved_slice.min_distance = min_d;
        Some(saved_slice)
    }

    fn generate_basic_slice(&mut self) -> ScanSlice {
        if let Some(ref target_pos_cell) = self.target_pos {
            let target_pos = *target_pos_cell.borrow();
            if let Some(pos) = target_pos {
                let num_slices = 6 * (TAU / self.base_search_width).round() as usize;
                let slice_width = self.biased_scan_width / num_slices as f64;

                if self.slice_index >= num_slices {
                    self.slice_index = 0;
                }

                let target_angle = (pos - position()).angle();
                let start_angle = target_angle - self.biased_scan_width / 2.0;
                let angle = start_angle + (self.slice_index as f64 + 0.5) * slice_width;

                self.slice_index += 1;

                self.swept_angle = 0.0;
                self.hit_seen_in_cycle = false;

                return ScanSlice {
                    angle,
                    width: slice_width,
                    min_distance: 0.0,
                    max_distance: self.max_distance,
                };
            }
        }

        self.slice_index = 0;

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

fn project_boundary_points(contact: &Contact) -> Option<[Vec2; 4]> {
    let pts = contact.scan_boundary_points?;
    let mut vels = contact.scan_boundary_vels?;
    let center = (pts[0] + pts[1] + pts[2] + pts[3]) / 4.0;
    let stats = contact.class.default_stats();
    let mut max_acc = stats
        .max_forward_acceleration
        .max(stats.max_backward_acceleration)
        .max(stats.max_lateral_acceleration);
    if contact.class == Class::Fighter || contact.class == Class::Missile {
        max_acc += 100.0;
    }
    let mut projected = pts;
    for i in 0..4 {
        let diff = pts[i] - center;
        let dir = if diff.length() > 1e-6 {
            diff.normalize()
        } else {
            Vec2::new(0.0, 0.0)
        };
        vels[i] += dir * (max_acc * 2.0f64.sqrt() * TICK_LENGTH);
        projected[i] += vels[i] * TICK_LENGTH;
    }
    Some(projected)
}

fn is_point_in_beam(
    pt: Vec2,
    beam_angle: f64,
    beam_width: f64,
    beam_min_d: f64,
    beam_max_d: f64,
    ship_pos: Vec2,
) -> bool {
    let to_pt = pt - ship_pos;
    let dist = to_pt.length();
    if dist < beam_min_d || dist > beam_max_d {
        return false;
    }
    let diff = angle_diff(beam_angle, to_pt.angle());
    diff.abs() <= beam_width / 2.0
}

fn cross_2d(v1: Vec2, v2: Vec2) -> f64 {
    v1.x * v2.y - v1.y * v2.x
}

fn intersect_segments(a: Vec2, b: Vec2, c: Vec2, d: Vec2) -> Option<Vec2> {
    let r = b - a;
    let s = d - c;
    let r_cross_s = cross_2d(r, s);
    if r_cross_s.abs() < 1e-12 {
        return None;
    }
    let t = cross_2d(c - a, s) / r_cross_s;
    let u = cross_2d(c - a, r) / r_cross_s;
    if t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0 {
        Some(a + r * t)
    } else {
        None
    }
}

fn get_polygon_candidates_in_beam(
    pts: [Vec2; 4],
    beam_angle: f64,
    beam_width: f64,
    beam_min_d: f64,
    beam_max_d: f64,
    ship_pos: Vec2,
) -> Vec<Vec2> {
    let edges = [
        (pts[0], pts[1]),
        (pts[1], pts[3]),
        (pts[3], pts[2]),
        (pts[2], pts[0]),
    ];

    let left_dir = Vec2::new(
        (beam_angle + beam_width / 2.0).cos(),
        (beam_angle + beam_width / 2.0).sin(),
    );
    let right_dir = Vec2::new(
        (beam_angle - beam_width / 2.0).cos(),
        (beam_angle - beam_width / 2.0).sin(),
    );

    let left_min = ship_pos + left_dir * beam_min_d;
    let left_max = ship_pos + left_dir * beam_max_d;
    let right_min = ship_pos + right_dir * beam_min_d;
    let right_max = ship_pos + right_dir * beam_max_d;

    let beam_boundaries = [
        (left_min, left_max),
        (right_min, right_max),
        (left_min, right_min),
        (left_max, right_max),
    ];

    let mut candidates = Vec::new();

    for (a, b) in &edges {
        // 1. Check endpoints of edge segment
        if is_point_in_beam(*a, beam_angle, beam_width, beam_min_d, beam_max_d, ship_pos) {
            candidates.push(*a);
        }
        if is_point_in_beam(*b, beam_angle, beam_width, beam_min_d, beam_max_d, ship_pos) {
            candidates.push(*b);
        }

        // 2. Check projection of ship onto the edge segment
        let d = *b - *a;
        let d_sq = d.dot(d);
        if d_sq > 1e-12 {
            let t = (*a - ship_pos).dot(d) / -d_sq;
            let t_clamped = t.clamp(0.0, 1.0);
            let proj = *a + d * t_clamped;
            if is_point_in_beam(
                proj, beam_angle, beam_width, beam_min_d, beam_max_d, ship_pos,
            ) {
                candidates.push(proj);
            }
        }

        // 3. Check intersections of edge segment with 4 beam boundary segments
        for (c, d) in &beam_boundaries {
            if let Some(pt) = intersect_segments(*a, *b, *c, *d) {
                candidates.push(pt);
            }
        }
    }
    candidates
}

fn find_closest_polygon_point_in_beam(
    pts: [Vec2; 4],
    beam_angle: f64,
    beam_width: f64,
    beam_min_d: f64,
    beam_max_d: f64,
    ship_pos: Vec2,
) -> Option<Vec2> {
    let candidates = get_polygon_candidates_in_beam(
        pts, beam_angle, beam_width, beam_min_d, beam_max_d, ship_pos,
    );
    let mut best_pt = None;
    let mut min_d = f64::MAX;
    for pt in candidates {
        let d = ship_pos.distance(pt);
        if d < min_d {
            min_d = d;
            best_pt = Some(pt);
        }
    }
    best_pt
}

fn find_furthest_polygon_point_in_beam(
    pts: [Vec2; 4],
    beam_angle: f64,
    beam_width: f64,
    beam_min_d: f64,
    beam_max_d: f64,
    ship_pos: Vec2,
) -> Option<Vec2> {
    let candidates = get_polygon_candidates_in_beam(
        pts, beam_angle, beam_width, beam_min_d, beam_max_d, ship_pos,
    );
    let mut best_pt = None;
    let mut max_d = -f64::MAX;
    for pt in candidates {
        let d = ship_pos.distance(pt);
        if d > max_d {
            max_d = d;
            best_pt = Some(pt);
        }
    }
    best_pt
}

impl ScanSliceGenerator for DefaultScanSliceGenerator {
    fn notify_hit(&mut self) {
        self.hit_seen_in_cycle = true;
    }

    fn notify_non_missile_contact(&mut self, has: bool) {
        self.has_non_missile_contact = has;
    }

    fn search_width(&self) -> f64 {
        self.search_width
    }

    fn next_slice(&mut self, contacts: &[Contact]) -> ScanSlice {
        loop {
            let mut slice = if let Some(avoided_id) = self.avoided_contact_id {
                let Some(s) = self.get_avoided_slice(avoided_id, contacts) else {
                    self.avoided_contact_id = None;
                    self.current_slice = None;
                    continue;
                };
                s
            } else {
                self.generate_basic_slice()
            };

            if slice.max_distance - slice.min_distance < 100.0 {
                self.avoided_contact_id = None;
                self.current_slice = None;
                self.last_scan_heading = Some(slice.angle);
                continue;
            }

            let next_our_pos = position() + velocity() * TICK_LENGTH;
            let mut retained = Vec::new();
            for contact in contacts {
                if Some(contact.id) == self.avoided_contact_id {
                    continue;
                }
                let Some(proj_pts) = project_boundary_points(contact) else {
                    continue;
                };
                if let Some(closest_pt) = find_closest_polygon_point_in_beam(
                    proj_pts,
                    slice.angle,
                    slice.width,
                    slice.min_distance,
                    slice.max_distance,
                    next_our_pos,
                ) {
                    debug!("avoid {}", contact.id);
                    retained.push((contact.id, closest_pt));
                }
            }

            if retained.is_empty() {
                self.avoided_contact_id = None;
                self.current_slice = None;
                return slice;
            }

            let mut closest_point = None;
            let mut closest_dist = f64::MAX;
            let mut closest_contact_id = 0;

            for (cid, pt) in retained {
                let d = next_our_pos.distance(pt);
                if d < closest_dist {
                    closest_dist = d;
                    closest_point = Some(pt);
                    closest_contact_id = cid;
                }
            }

            let Some(pt) = closest_point else {
                self.avoided_contact_id = None;
                self.current_slice = None;
                return slice;
            };

            let dist = next_our_pos.distance(pt);
            if dist - slice.min_distance < 100.0 {
                self.avoided_contact_id = None;
                self.current_slice = None;
                return slice;
            }

            slice.max_distance = dist;
            self.avoided_contact_id = Some(closest_contact_id);
            let mut saved_slice = slice;
            saved_slice.max_distance = self.max_distance;
            self.current_slice = Some(saved_slice);
            return slice;
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
    pub priority_target_frequencies: Vec<(u32, f64)>,
    scan_ticks: u32,
    last_scan_heading: Option<f64>,
    pub slice_generator: Box<dyn ScanSliceGenerator>,
    pub jamming_mode: bool,
    pub new_missile_scan_ticks: u32,
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
            priority_target_frequencies: Vec::new(),
            scan_ticks: 0,
            last_scan_heading: None,
            slice_generator: Box::new(DefaultScanSliceGenerator::new(search_width, max_distance)),
            jamming_mode: false,
            new_missile_scan_ticks: 8,
        }
    }

    fn is_priority_target(&self, contact_id: u32) -> bool {
        self.priority_target_frequencies
            .iter()
            .any(|&(id, _)| id == contact_id)
    }

    fn tracking_interval(&self, contact: &Contact) -> u32 {
        let is_new_missile =
            contact.class == Class::Missile && contact.missile_scan_ticks_remaining > 0;
        if is_new_missile {
            return 1;
        }

        if let Some(&(_, freq)) = self
            .priority_target_frequencies
            .iter()
            .find(|&&(id, _)| id == contact.id)
        {
            let ticks = (freq / TICK_LENGTH).round() as u32;
            return ticks.max(1);
        }

        let is_priority = contact.provisional || contact.prioritize_scan;
        if is_priority { 6 } else { 20 }
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

    pub fn process_scan_hit(&mut self, c: ScanResult, slice: Option<ScanSlice>) -> u32 {
        let current_t = current_tick();
        for contact in &mut self.contacts {
            contact.predict(current_t);
        }
        let (pos_unc, vel_unc) = measurement_uncertainty(&c);
        let scan_ci_radius = ci_multiplier(0.995) * pos_unc;

        let matched = self.find_best_matching_contact(&c, scan_ci_radius, slice, current_t, 1.0);

        let returned_id = if let Some((best_id, _)) = matched {
            // Otherwise we have a match
            let contact = self
                .contacts
                .iter_mut()
                .find(|co| co.id == best_id)
                .unwrap();

            let prev_unc = contact
                .prev_scan_pos_uncertainty
                .unwrap_or(contact.pos_uncertainty);
            let predicted_unc = contact.current_pos_uncertainty();
            contact.update_with_measurement(c.position, c.velocity, pos_unc, vel_unc, current_t);
            if let Some(s) = slice {
                contact.update_scan_boundary(s);
            }
            let current_unc = contact.pos_uncertainty;
            let _pct_improvement = if predicted_unc > 0.0 {
                (1.0 - current_unc / predicted_unc) * 100.0
            } else {
                0.0
            };
            if contact.prioritize_scan {
                let current_unc = contact.pos_uncertainty;
                let shrink_ratio = current_unc / prev_unc;
                let improvement_pct = (1.0 - shrink_ratio) * 100.0;
                if improvement_pct < 5.0 {
                    contact.low_improvement_consecutive_scans += 1;
                } else {
                    contact.low_improvement_consecutive_scans = 0;
                }

                if contact.low_improvement_consecutive_scans >= 2 {
                    contact.prioritize_scan = false;
                }
            }
            contact.prev_scan_pos_uncertainty = Some(contact.pos_uncertainty);
            contact.tracking_retry_count = 0;
            if c.snr == RADIO_PING_SNR {
                contact.provisional = false; // Confirm immediately if it is a radio ping
            }
            if let Some(s) = slice {
                contact.last_beam_width = Some(s.width);
                contact.last_beam_center = Some(s.angle);
                let d_last = position().distance(contact.position);
                contact.last_beam_center_pos =
                    Some(position() + vec2(s.angle.cos() * d_last, s.angle.sin() * d_last));
            }
            if contact.missile_scan_ticks_remaining > 0 {
                contact.missile_scan_ticks_remaining -= 1;
            }
            best_id
        } else {
            let dist_to_radar = position().distance(c.position);
            if let Some(s) = slice {
                if !is_within_reliable_range(c.class, dist_to_radar, s.width) {
                    return 0;
                }
            }
            let last_scanned = current_t;
            let cov_x = Contact::initial_cov(pos_unc, vel_unc, c.class);
            let cov_y = Contact::initial_cov(pos_unc, vel_unc, c.class);

            let new_id = self.next_contact_id;
            let discovery_beam_width = slice.map(|s| s.width).unwrap_or_else(|| self.slice_generator.search_width());
            self.contacts.push(Contact {
                id: new_id,
                kinematic: KinematicState::new(
                    c.class,
                    c.position,
                    c.velocity,
                    Vec2::new(0.0, 0.0),
                    last_scanned,
                ),
                last_measurement_tick: last_scanned,
                pos_uncertainty: pos_unc,
                vel_uncertainty: vel_unc,
                provisional: c.snr != RADIO_PING_SNR, // Confirm immediately if it is a radio ping
                tracking_retry_count: 0,
                confirmation_attempts: 0,
                unscanned_in_range_ticks: 0,
                p_cov_x: cov_x,
                p_cov_y: cov_y,
                prioritize_scan: c.class == Class::Missile,
                prev_scan_pos_uncertainty: if c.class == Class::Missile {
                    Some(pos_unc)
                } else {
                    None
                },
                low_improvement_consecutive_scans: 0,
                last_beam_width: slice.map(|s| s.width),
                last_beam_center: slice.map(|s| s.angle),
                last_beam_center_pos: slice.map(|s| {
                    let d_last = position().distance(c.position);
                    position() + vec2(s.angle.cos() * d_last, s.angle.sin() * d_last)
                }),
                missile_scan_ticks_remaining: if c.class == Class::Missile {
                    self.new_missile_scan_ticks
                } else {
                    0
                },
                scan_boundary_points: None,
                scan_boundary_vels: None,
                discovery_beam_width,
            });
            self.next_contact_id += 1;
            debug!("Discovered new contact {}", new_id);
            new_id
        };

        // Immediately remove enemy missiles or torpedoes moving away from us
        self.contacts.retain(|contact| {
            if contact.class == Class::Missile || contact.class == Class::Torpedo {
                let r = contact.current_position() - position();
                let v_rel = contact.current_velocity() - velocity();
                let retain = r.dot(v_rel) <= 0.0;
                if !retain {
                    debug!("Not retaining missile -- negative closing velocity");
                }
                retain
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
            let sum = self
                .non_provisional_contacts
                .iter()
                .fold(Vec2::new(0.0, 0.0), |acc, c| acc + c.current_position());
            sum / self.non_provisional_contacts.len() as f64
        }
    }

    fn num_radars(&self) -> usize {
        if class() == Class::Cruiser { 2 } else { 1 }
    }

    pub fn update(&mut self) {
        self.predict();
        let hit_seen_this_tick = self.process_scans();
        self.cleanup();
        self.generate_new_scans(hit_seen_this_tick);
    }

    fn predict(&mut self) {
        let current_t = current_tick();
        for contact in &mut self.contacts {
            contact.predict(current_t);
        }
    }

    fn process_scans(&mut self) -> bool {
        let current_t = current_tick();
        let num_radars = self.num_radars();
        let mut hit_seen_this_tick = false;

        // 1. Process scan results from previous tick depending on radar_states
        let mut scan_results = Vec::new();
        let mut scan_returned_none = vec![false; num_radars];
        for i in 0..num_radars {
            select_radar(i);
            if let Some(r) = scan() {
                hit_seen_this_tick = true;
                // Draw 99.99% confidence interval circle for this radar hit
                let (pos_unc, _) = measurement_uncertainty(&r);
                let radius = ci_multiplier(0.995) * pos_unc;
                draw_polygon(r.position, radius, 32, 0.0, rgb(255, 255, 0));

                scan_results.push((i, r));
            } else {
                scan_returned_none[i] = true;
                let RadarState::Tracking { contact_id } = self.radar_states[i] else {
                    continue;
                };
                let Some(intended_contact) =
                    self.contacts.iter_mut().find(|co| co.id == contact_id)
                else {
                    continue;
                };
                intended_contact.tracking_retry_count += 1;
                intended_contact.expand_scan_boundary(0.1);
                debug!(
                    "Radar {} tracking scan for contact {} returned None. Incrementing retry count to {}.",
                    i, contact_id, intended_contact.tracking_retry_count
                );
            }
        }

        for (i, c) in scan_results {
            select_radar(i);
            let state = self.radar_states[i];
            let slice = self.prev_slices[i];
            match state {
                RadarState::Scanning | RadarState::Jamming { .. } => {
                    self.process_scan_hit(c, slice);
                }
                RadarState::Tracking { contact_id } => {
                    let mut best_match_id = None;
                    let mut best_dist = f64::MAX;

                    // Calculate scan's hit position 99.99% CI radius
                    let (pos_unc, vel_unc) = measurement_uncertainty(&c);
                    let scan_ci_radius = ci_multiplier(0.995) * pos_unc;

                    if let Some((id, dist)) =
                        self.find_best_matching_contact(&c, scan_ci_radius, slice, current_t, 1.5)
                    {
                        best_match_id = Some(id);
                        best_dist = dist;
                    }

                    if let Some(best_id) = best_match_id {
                        {
                            let contact = self
                                .contacts
                                .iter_mut()
                                .find(|co| co.id == best_id)
                                .unwrap();
                            let ci_radius = contact.ci_mult() * contact.current_pos_uncertainty();
                            let stats = contact.class.default_stats();
                            let mut max_acc = stats
                                .max_forward_acceleration
                                .max(stats.max_backward_acceleration)
                                .max(stats.max_lateral_acceleration);
                            if contact.class == Class::Fighter || contact.class == Class::Missile {
                                max_acc += 100.0;
                            }
                            let dt_sec = current_t.wrapping_sub(contact.last_measurement_tick)
                                as f64
                                * TICK_LENGTH;
                            let fallback = 0.5 * max_acc * dt_sec * dt_sec;
                            let gate_radius = 1.5 * (ci_radius.max(10.0) + fallback);

                            if best_dist > ci_radius.max(10.0) && best_dist > 20.0 {
                                if best_dist > gate_radius {
                                    debug!(
                                        "Tracked scan hit associated to contact {} was outside contact's CI and moved position by {:.1}m (>20m), associated because it is within scan's 99.99% CI ({:.1}m)",
                                        contact.id,
                                        best_dist,
                                        1.5 * scan_ci_radius
                                    );
                                } else {
                                    debug!(
                                        "Tracked scan hit associated to contact {} was outside CI and moved position by {:.1}m (>20m), associated due to dynamic gating fallback ({:.1}m based on max accel {:.1}m/s^2 and dt={:.3}s)",
                                        contact.id, best_dist, fallback, max_acc, dt_sec
                                    );
                                }
                            }

                            let prev_unc = contact
                                .prev_scan_pos_uncertainty
                                .unwrap_or(contact.pos_uncertainty);
                            let predicted_unc = contact.current_pos_uncertainty();
                            contact.update_with_measurement(
                                c.position, c.velocity, pos_unc, vel_unc, current_t,
                            );
                            if let Some(s) = slice {
                                contact.update_scan_boundary(s);
                            }
                            let current_unc = contact.pos_uncertainty;
                            let _pct_improvement = if predicted_unc > 0.0 {
                                (1.0 - current_unc / predicted_unc) * 100.0
                            } else {
                                0.0
                            };
                            if contact.prioritize_scan {
                                let current_unc = contact.pos_uncertainty;
                                let shrink_ratio = current_unc / prev_unc;
                                let improvement_pct = (1.0 - shrink_ratio) * 100.0;
                                if improvement_pct < 5.0 {
                                    contact.low_improvement_consecutive_scans += 1;
                                } else {
                                    contact.low_improvement_consecutive_scans = 0;
                                }

                                if contact.low_improvement_consecutive_scans >= 2 {
                                    contact.prioritize_scan = false;
                                }
                            }
                            contact.prev_scan_pos_uncertainty = Some(contact.pos_uncertainty);
                            contact.tracking_retry_count = 0;
                            if let Some(s) = self.prev_slices[i] {
                                contact.last_beam_width = Some(s.width);
                                contact.last_beam_center = Some(s.angle);
                                let d_last = position().distance(contact.position);
                                contact.last_beam_center_pos = Some(
                                    position()
                                        + vec2(s.angle.cos() * d_last, s.angle.sin() * d_last),
                                );
                            }
                            if contact.missile_scan_ticks_remaining > 0 {
                                contact.missile_scan_ticks_remaining -= 1;
                            }
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
                            let contact = self
                                .contacts
                                .iter_mut()
                                .find(|co| co.id == best_id)
                                .unwrap();
                            if contact.provisional {
                                contact.confirmation_attempts += 1;
                                if is_distinct {
                                    contact.provisional = false;
                                    debug!(
                                        "Confirmed contact {} because it is definitely distinct from lower-numbered contacts (>30m)",
                                        best_id
                                    );
                                } else {
                                    debug!(
                                        "Contact {} remains unconfirmed (distance to a lower-numbered contact is <= 30m)",
                                        best_id
                                    );
                                }
                            }

                            if contact.provisional && contact.confirmation_attempts >= 3 {
                                debug!(
                                    "Dropping contact {} because it could not be confirmed after 3 tracking attempts",
                                    best_id
                                );
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
                                debug!(
                                    "Tracked contact {} distance to closest lower ID contact: {:.1}m",
                                    best_id, d
                                );
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
                                debug!(
                                    "Dropping tracked contact {} (higher ID) because it is within 15m of contact with lower ID",
                                    best_id
                                );
                                self.contacts.retain(|other| other.id != best_id);
                            }
                        }
                    } else {
                        // Create a new contact
                        let (pos_unc, vel_unc) = measurement_uncertainty(&c);
                        let dist = position().distance(c.position);

                        let mut should_add = true;
                        if let Some(s) = slice {
                            if !is_within_reliable_range(c.class, dist, s.width) {
                                should_add = false;
                            }
                        }

                        if should_add {
                            let ci_radius = ci_multiplier(0.995) * pos_unc;

                            let includes_preexisting = self.contacts.iter().any(|co| {
                                co.class == c.class
                                    && co.current_position().distance(c.position) <= ci_radius
                            });

                            if !includes_preexisting {
                                let last_scanned = current_t;

                                let cov_x = Contact::initial_cov(pos_unc, vel_unc, c.class);
                                let cov_y = Contact::initial_cov(pos_unc, vel_unc, c.class);

                                let discovery_beam_width = slice.map(|s| s.width).unwrap_or_else(|| self.slice_generator.search_width());
                                self.contacts.push(Contact {
                                    id: self.next_contact_id,
                                    kinematic: KinematicState::new(
                                        c.class,
                                        c.position,
                                        c.velocity,
                                        Vec2::new(0.0, 0.0),
                                        last_scanned,
                                    ),
                                    last_measurement_tick: last_scanned,
                                    pos_uncertainty: pos_unc,
                                    vel_uncertainty: vel_unc,
                                    provisional: true,
                                    tracking_retry_count: 0,
                                    confirmation_attempts: 0,
                                    unscanned_in_range_ticks: 0,
                                    p_cov_x: cov_x,
                                    p_cov_y: cov_y,
                                    prioritize_scan: c.class == Class::Missile,
                                    prev_scan_pos_uncertainty: if c.class == Class::Missile {
                                        Some(pos_unc)
                                    } else {
                                        None
                                    },
                                    low_improvement_consecutive_scans: 0,
                                    last_beam_width: slice.map(|s| s.width),
                                    last_beam_center: slice.map(|s| s.angle),
                                    last_beam_center_pos: slice.map(|s| {
                                        let d_last = position().distance(c.position);
                                        position()
                                            + vec2(s.angle.cos() * d_last, s.angle.sin() * d_last)
                                    }),
                                    missile_scan_ticks_remaining: if c.class == Class::Missile {
                                        self.new_missile_scan_ticks
                                    } else {
                                        0
                                    },
                                    scan_boundary_points: None,
                                    scan_boundary_vels: None,
                                    discovery_beam_width,
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
                        let Some(contact) = self.contacts.iter_mut().find(|co| co.id == contact_id)
                        else {
                            continue;
                        };
                        contact.expand_scan_boundary(0.1);
                        let expected_pos = contact.current_position();
                        let ci_radius = contact.ci_mult() * contact.current_pos_uncertainty();
                        let stats = contact.class.default_stats();
                        let mut max_acc = stats
                            .max_forward_acceleration
                            .max(stats.max_backward_acceleration)
                            .max(stats.max_lateral_acceleration);
                        if contact.class == Class::Fighter || contact.class == Class::Missile {
                            max_acc += 100.0;
                        }
                        let dt_sec = current_t.wrapping_sub(contact.last_measurement_tick) as f64
                            * TICK_LENGTH;
                        let fallback = 0.5 * max_acc * dt_sec * dt_sec;
                        let gate_radius = ci_radius.max(10.0) + fallback;
                        draw_polygon(expected_pos, gate_radius, 16, 0.0, rgb(255, 0, 0)); // Red color
                    }
                }
            }
        }

        hit_seen_this_tick
    }

    fn cleanup(&mut self) {
        let current_t = current_tick();
        // Update unscanned_in_range_ticks
        for contact in &mut self.contacts {
            if contact.last_measurement_tick == current_t {
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
                if contact.last_measurement_tick != current_t {
                    keep = false;
                    let next_pos = contact.position_at(current_t);
                    debug!(
                        "removed {}: pos=None expect=({:.1}, {:.1}) (failed tracking 3 times)",
                        contact.id, next_pos.x, next_pos.y
                    );
                }
            }
            keep
        });

        // 3. Prune old contacts (timeout after 120 ticks / 2 seconds of being in range and unscanned)
        self.contacts.retain(|c| {
            let keep = c.unscanned_in_range_ticks <= 120;
            if !keep {
                let current_pos = c.current_position();
                debug!(
                    "removed {}: pos=None expect=({:.1}, {:.1}) gate=None",
                    c.id, current_pos.x, current_pos.y
                );
            }
            keep
        });
    }

    fn generate_new_scans(&mut self, hit_seen_this_tick: bool) {
        let current_t = current_tick();
        let num_radars = self.num_radars();
        if hit_seen_this_tick {
            self.slice_generator.notify_hit();
        }
        let has_non_missile = self.contacts.iter().any(|c| c.class != Class::Missile);
        self.slice_generator
            .notify_non_missile_contact(has_non_missile);

        // 4. Generate jobs for next tick
        let mut jobs = Vec::new();

        for job in self.tracking_jobs() {
            if jobs.len() >= num_radars {
                break;
            }
            jobs.push(job);
        }

        if self.jamming_mode && jobs.len() < num_radars {
            for contact in &self.contacts {
                if jobs.len() >= num_radars {
                    break;
                }
                let Some(mut job) = self.generate_tracking_scan(contact, current_t) else {
                    continue;
                };
                job.state = RadarState::Jamming {
                    contact_id: contact.id,
                };
                jobs.push(job);
            }
        }

        while jobs.len() < num_radars {
            let slice = self.slice_generator.next_slice(&self.contacts);
            let job = RadarJob {
                angle: slice.angle,
                width: slice.width,
                min_distance: slice.min_distance,
                max_distance: slice.max_distance,
                state: RadarState::Scanning,
            };
            jobs.push(job);
        }

        // Assign jobs to available radars
        let mut next_radar = 0;
        for job in jobs {
            let r = next_radar;
            next_radar += 1;

            select_radar(r);
            set_radar_heading(job.angle);
            set_radar_width(job.width);
            set_radar_min_distance(job.min_distance);
            set_radar_max_distance(job.max_distance);

            match job.state {
                RadarState::Jamming { .. } => set_radar_ecm_mode(EcmMode::Noise),
                _ => set_radar_ecm_mode(EcmMode::None),
            }

            self.radar_states[r] = job.state;
            self.prev_slices[r] = Some(ScanSlice {
                angle: job.angle,
                width: job.width,
                min_distance: job.min_distance,
                max_distance: job.max_distance,
            });

            if let RadarState::Scanning = job.state {
                self.last_scan_heading = Some(job.angle);
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
            let draw_ci = |mult: f64| {
                let radius = ci_multiplier(mult) * contact.current_pos_uncertainty();
                let dist_from_orange = 1.0 - mult;
                let color = rgb(
                    255,
                    165 - (255.0 * dist_from_orange) as u8,
                    0 + (255.0 * dist_from_orange) as u8,
                );
                draw_polygon(contact.current_position(), radius, 16, 0.0, color);
            };
            // draw_ci(0.995);
            // draw_ci(0.99);
            // draw_ci(0.95);

            let text_pos = contact.current_position() + vec2(0.0, radius + 20.0);
            draw_text!(text_pos, rgb(255, 165, 0), "cid: {}", contact.id);

            if let Some(pts) = contact.scan_boundary_points {
                draw_line(pts[0], pts[1], rgb(0, 255, 255));
                draw_line(pts[2], pts[3], rgb(0, 255, 255));
                draw_line(pts[0], pts[2], rgb(0, 255, 255));
                draw_line(pts[1], pts[3], rgb(0, 255, 255));
            }
        }

        // Cache non-provisional contacts
        self.non_provisional_contacts = self
            .contacts
            .iter()
            .filter(|c| !c.provisional)
            .cloned()
            .collect();
    }

    pub fn tracking_jobs(&self) -> impl Iterator<Item = RadarJob> + '_ {
        let current_t = current_tick();
        let mut tracking_contacts = Vec::new();

        if self.jamming_mode && !self.priority_target_frequencies.is_empty() {
            for contact in &self.contacts {
                let is_new_missile =
                    contact.class == Class::Missile && contact.missile_scan_ticks_remaining > 0;
                let is_priority_target = self.is_priority_target(contact.id);
                if !is_new_missile && !is_priority_target {
                    continue;
                }
                let priority_group = if is_new_missile { 0 } else { 1 };
                let interval = self.tracking_interval(contact);
                let next_track_tick = if contact.provisional {
                    current_t
                } else {
                    contact
                        .last_measurement_tick
                        .wrapping_add(interval * (1 + contact.tracking_retry_count))
                };
                tracking_contacts.push((priority_group, next_track_tick, contact));
            }
            tracking_contacts.sort_by_key(|&(priority_group, next_track_tick, _)| {
                (priority_group, next_track_tick)
            });
        } else {
            for contact in &self.contacts {
                let is_new_missile =
                    contact.class == Class::Missile && contact.missile_scan_ticks_remaining > 0;
                let is_priority = self.is_priority_target(contact.id)
                    || contact.provisional
                    || contact.prioritize_scan;
                let priority_group = if is_new_missile {
                    0
                } else if is_priority {
                    1
                } else {
                    2
                };
                let interval = self.tracking_interval(contact);
                let next_track_tick = if contact.provisional {
                    current_t
                } else {
                    contact
                        .last_measurement_tick
                        .wrapping_add(interval * (1 + contact.tracking_retry_count))
                };
                tracking_contacts.push((priority_group, next_track_tick, contact));
            }
            tracking_contacts.sort_by_key(|&(priority_group, next_track_tick, _)| {
                (priority_group, next_track_tick)
            });
        }

        tracking_contacts
            .into_iter()
            .filter_map(move |(_, next_track_tick, contact)| {
                if next_track_tick <= current_t {
                    self.generate_tracking_scan(contact, current_t)
                } else {
                    None
                }
            })
    }

    fn find_best_matching_contact(
        &self,
        c: &ScanResult,
        scan_ci_radius: f64,
        slice: Option<ScanSlice>,
        current_t: u32,
        gate_multiplier: f64,
    ) -> Option<(u32, f64)> {
        let mut best_match = None;
        let mut best_dist = f64::MAX;

        for contact in &self.contacts {
            if contact.class != c.class {
                continue;
            }

            if let Some(s) = slice {
                if let Some(pts) = contact.scan_boundary_points {
                    let mut min_pt_dist = f64::MAX;
                    let mut max_pt_dist = -f64::MAX;
                    let mut min_rel_angle = f64::MAX;
                    let mut max_rel_angle = -f64::MAX;

                    for pt in &pts {
                        let to_pt = *pt - position();
                        let d_pt = to_pt.length();
                        min_pt_dist = min_pt_dist.min(d_pt);
                        max_pt_dist = max_pt_dist.max(d_pt);

                        let rel_angle = normalize_angle(to_pt.angle() - s.angle);
                        min_rel_angle = min_rel_angle.min(rel_angle);
                        max_rel_angle = max_rel_angle.max(rel_angle);
                    }

                    let d_overlap = max_pt_dist >= s.min_distance && min_pt_dist <= s.max_distance;
                    let a_overlap =
                        max_rel_angle >= -s.width / 2.0 && min_rel_angle <= s.width / 2.0;

                    if !(d_overlap && a_overlap) {
                        continue;
                    }
                }

                let gate_radius = contact.ci_mult() * contact.current_pos_uncertainty();
                let contact_pos = contact.current_position();
                let v = contact_pos - position();
                let contact_d = v.length();
                let contact_angle = v.angle();

                let d_overlap = contact_d + gate_radius >= s.min_distance
                    && contact_d - gate_radius <= s.max_distance;

                let a_overlap = if gate_radius >= contact_d {
                    true
                } else {
                    let gate_angle = gate_radius / contact_d;
                    let a_diff = angle_diff(s.angle, contact_angle).abs();
                    a_diff <= s.width / 2.0 + gate_angle
                };

                if !(d_overlap && a_overlap) {
                    continue;
                }
            }

            let expected_pos = contact.current_position();
            let dist = expected_pos.distance(c.position);
            let ci_radius = contact.ci_mult() * contact.current_pos_uncertainty();
            let stats = contact.class.default_stats();
            let mut max_acc = stats
                .max_forward_acceleration
                .max(stats.max_backward_acceleration)
                .max(stats.max_lateral_acceleration);
            if contact.class == Class::Fighter || contact.class == Class::Missile {
                max_acc += 100.0;
            }
            let dt_sec = current_t.wrapping_sub(contact.last_measurement_tick) as f64 * TICK_LENGTH;
            let fallback = 0.5 * max_acc * dt_sec * dt_sec;
            let gate_radius = gate_multiplier * (ci_radius.max(10.0) + fallback);

            // Count match if scan is within contact's CI (gate_radius) OR contact is within scan's CI (gate_multiplier * scan_ci_radius)
            let is_match = dist < gate_radius || dist < gate_multiplier * scan_ci_radius;
            if is_match && dist < best_dist {
                best_dist = dist;
                best_match = Some((contact.id, dist));
            }
        }

        best_match
    }

    fn is_contact_matched_by_beam(
        &self,
        contact: &Contact,
        beam: &RadarJob,
        current_t: u32,
    ) -> bool {
        let next_our_pos = position() + velocity() * TICK_LENGTH;
        let next_contact_pos = contact.position_at(current_t + 1);
        let v = next_contact_pos - next_our_pos;
        let contact_d = v.length();
        let contact_angle = v.angle();

        let next_pos_uncertainty = contact.pos_uncertainty_at(current_t + 1);
        let gate_radius = (contact.ci_mult() * next_pos_uncertainty).max(self.gate_radius);
        let gate_angle = gate_radius / contact_d;

        let d_overlap = contact_d + gate_radius >= beam.min_distance
            && contact_d - gate_radius <= beam.max_distance;
        let a_diff = angle_diff(beam.angle, contact_angle).abs();
        let a_overlap = a_diff <= beam.width / 2.0 + gate_angle;

        d_overlap && a_overlap
    }

    fn generate_tracking_scan(&self, contact: &Contact, current_t: u32) -> Option<RadarJob> {
        let next_pos = contact.position_at(current_t + 1);
        let next_our_pos = position() + velocity() * TICK_LENGTH;
        let mut d = next_our_pos.distance(next_pos);

        let mut target_pos = next_pos;
        if let (Some(prev_width), Some(center_pos)) =
            (contact.last_beam_width, contact.last_beam_center_pos)
        {
            let next_pos_uncertainty = contact.pos_uncertainty_at(current_t + 1);
            let ci_99_radius = ci_multiplier(0.99) * next_pos_uncertainty;
            let beam_width_at_target = d * prev_width;
            if ci_99_radius > beam_width_at_target {
                let dt_sec = (current_t + 1).wrapping_sub(contact.last_measurement_tick) as f64
                    * TICK_LENGTH;
                target_pos = center_pos + contact.velocity * dt_sec;
                d = next_our_pos.distance(target_pos);
            }
        }

        if !is_within_range(contact, d) {
            return None;
        }
        let target_angle = (target_pos - next_our_pos).angle();

        let next_pos_uncertainty = contact.pos_uncertainty_at(current_t + 1);
        let gate_radius = (contact.ci_mult() * next_pos_uncertainty).max(self.gate_radius);
        let max_width = self.tracking_width;

        let mut projected_pts = None;
        let mut boundary_dist_limits = None;
        let mut boundary_angle_limits = None;

        if let (Some(pts), Some(vels)) = (contact.scan_boundary_points, contact.scan_boundary_vels)
        {
            let mut proj_pts = [Vec2::new(0.0, 0.0); 4];
            for j in 0..4 {
                proj_pts[j] = pts[j] + vels[j] * TICK_LENGTH;
            }

            let mut min_pt_dist = f64::MAX;
            let mut max_pt_dist = -f64::MAX;
            for pt in &proj_pts {
                let d_pt = next_our_pos.distance(*pt);
                min_pt_dist = min_pt_dist.min(d_pt);
                max_pt_dist = max_pt_dist.max(d_pt);
            }

            let mut min_rel_angle = f64::MAX;
            let mut max_rel_angle = -f64::MAX;
            for pt in &proj_pts {
                let pt_angle = (*pt - next_our_pos).angle();
                let rel_angle = normalize_angle(pt_angle - target_angle);
                min_rel_angle = min_rel_angle.min(rel_angle);
                max_rel_angle = max_rel_angle.max(rel_angle);
            }

            projected_pts = Some(proj_pts);
            boundary_dist_limits = Some((min_pt_dist, max_pt_dist));
            boundary_angle_limits = Some((min_rel_angle, max_rel_angle));
        }

        let initial_job =
            if let (Some((min_pt_dist, max_pt_dist)), Some((min_rel_angle, max_rel_angle))) =
                (boundary_dist_limits, boundary_angle_limits)
            {
                let width = (max_rel_angle - min_rel_angle).max(0.005);
                let angle = normalize_angle(target_angle + (min_rel_angle + max_rel_angle) / 2.0);

                RadarJob {
                    angle,
                    width,
                    min_distance: min_pt_dist,
                    max_distance: max_pt_dist,
                    state: RadarState::Tracking {
                        contact_id: contact.id,
                    },
                }
            } else {
                let calculated_width = clamped_tracking_width(
                    contact,
                    d,
                    gate_radius,
                    next_pos_uncertainty,
                    max_width,
                );
                let min_distance =
                    (d - (contact.ci_mult() * next_pos_uncertainty).max(10.0)).max(0.0);
                let max_distance = d + (contact.ci_mult() * next_pos_uncertainty).max(10.0);

                RadarJob {
                    angle: target_angle,
                    width: calculated_width.clamp(0.005, max_width),
                    min_distance,
                    max_distance,
                    state: RadarState::Tracking {
                        contact_id: contact.id,
                    },
                }
            };

        let draw_job_beam = |job: &RadarJob, color, label: &str| {
            let left_dir = Vec2::new(
                (job.angle + job.width / 2.0).cos(),
                (job.angle + job.width / 2.0).sin(),
            );
            let right_dir = Vec2::new(
                (job.angle - job.width / 2.0).cos(),
                (job.angle - job.width / 2.0).sin(),
            );

            let l_min = next_our_pos + left_dir * job.min_distance;
            let l_max = next_our_pos + left_dir * job.max_distance;
            let r_min = next_our_pos + right_dir * job.min_distance;
            let r_max = next_our_pos + right_dir * job.max_distance;

            draw_line(l_min, l_max, color);
            draw_line(r_min, r_max, color);
            draw_line(l_min, r_min, color);
            draw_line(l_max, r_max, color);

            draw_text!(l_max, color, "{}", label);
        };

        if let (Some((min_pt_dist, max_pt_dist)), Some((min_rel_angle, max_rel_angle))) =
            (boundary_dist_limits, boundary_angle_limits)
        {
            let original_width = max_rel_angle - min_rel_angle;
            if original_width > contact.discovery_beam_width {
                let width = (0.6 * original_width).max(0.005);
                let angle = match contact.tracking_retry_count {
                    0 => target_angle,
                    1 => normalize_angle(target_angle + max_rel_angle - 0.3 * original_width),
                    2 => normalize_angle(target_angle + min_rel_angle + 0.3 * original_width),
                    _ => target_angle,
                };
                let job = RadarJob {
                    angle,
                    width,
                    min_distance: min_pt_dist,
                    max_distance: max_pt_dist,
                    state: RadarState::Tracking {
                        contact_id: contact.id,
                    },
                };
                draw_job_beam(
                    &job,
                    rgb(0, 255, 0),
                    &format!("Selected: Bisected (cid {})", contact.id),
                );
                return Some(job);
            }
        }

        // Draw 99.9% CI circle
        let ci_999_radius = ci_multiplier(0.999) * next_pos_uncertainty;
        draw_polygon(next_pos, ci_999_radius, 32, 0.0, rgb(255, 0, 0));
        draw_text!(
            next_pos + vec2(0.0, ci_999_radius + 15.0),
            rgb(255, 0, 0),
            "99.9% CI (cid {})",
            contact.id
        );

        // Draw projected boundary points
        if let Some(projected_pts) = projected_pts {
            let color = rgb(0, 255, 255); // Cyan
            draw_line(projected_pts[0], projected_pts[1], color);
            draw_line(projected_pts[1], projected_pts[3], color);
            draw_line(projected_pts[3], projected_pts[2], color);
            draw_line(projected_pts[2], projected_pts[0], color);
            draw_text!(
                projected_pts[0],
                color,
                "Proj Boundary (cid {})",
                contact.id
            );
        }

        let constrain_job = |job: &mut RadarJob| {
            if let (Some((min_pt_dist, max_pt_dist)), Some((min_rel_angle, max_rel_angle))) =
                (boundary_dist_limits, boundary_angle_limits)
            {
                // 1. Distance constraints
                job.min_distance = job.min_distance.max(min_pt_dist);
                job.max_distance = job.max_distance.min(max_pt_dist).max(job.min_distance);

                // 2. Angular constraints
                let job_left = normalize_angle((job.angle + job.width / 2.0) - target_angle);
                let job_right = normalize_angle((job.angle - job.width / 2.0) - target_angle);

                let new_rel_left = job_left.min(max_rel_angle);
                let new_rel_right = job_right.max(min_rel_angle);

                let w_min = 0.005;
                let (new_width, new_center_rel) = if new_rel_left - new_rel_right < w_min {
                    (w_min, (min_rel_angle + max_rel_angle) / 2.0)
                } else {
                    (
                        new_rel_left - new_rel_right,
                        (new_rel_left + new_rel_right) / 2.0,
                    )
                };

                job.width = new_width;
                job.angle = normalize_angle(target_angle + new_center_rel);
            }
        };

        draw_job_beam(
            &initial_job,
            rgb(255, 255, 0),
            &format!("Initial (cid {})", contact.id),
        );

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
            let dist_to_target = next_other_pos.distance(target_pos);
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
            draw_job_beam(
                &initial_job,
                rgb(0, 255, 0),
                &format!("Selected: Initial (cid {})", contact.id),
            );
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
        candidates.push((initial_job, "Initial".to_string()));

        // a. Left contact midpoint
        if let Some(left_contact) = closest_left_contact {
            let left_angle = (left_contact.position_at(current_t + 1) - next_our_pos).angle();
            let angle_diff_val = angle_diff(target_angle, left_angle);
            let mid_angle = target_angle + angle_diff_val / 2.0;
            let initial_right_limit = target_angle - initial_job.width / 2.0;
            let width_raw = angle_diff(initial_right_limit, mid_angle);
            let clamped_width = width_raw.clamp(0.005, max_width);
            let new_angle = mid_angle - clamped_width / 2.0;
            candidates.push((
                RadarJob {
                    angle: new_angle,
                    width: clamped_width,
                    min_distance: initial_job.min_distance,
                    max_distance: initial_job.max_distance,
                    state: RadarState::Tracking {
                        contact_id: contact.id,
                    },
                },
                "Left Mid".to_string(),
            ));
        }

        // b. Right contact midpoint
        if let Some(right_contact) = closest_right_contact {
            let right_angle = (right_contact.position_at(current_t + 1) - next_our_pos).angle();
            let angle_diff_val = angle_diff(target_angle, right_angle);
            let mid_angle = target_angle + angle_diff_val / 2.0;
            let initial_left_limit = target_angle + initial_job.width / 2.0;
            let width_raw = angle_diff(mid_angle, initial_left_limit);
            let clamped_width = width_raw.clamp(0.005, max_width);
            let new_angle = mid_angle + clamped_width / 2.0;
            candidates.push((
                RadarJob {
                    angle: new_angle,
                    width: clamped_width,
                    min_distance: initial_job.min_distance,
                    max_distance: initial_job.max_distance,
                    state: RadarState::Tracking {
                        contact_id: contact.id,
                    },
                },
                "Right Mid".to_string(),
            ));
        }

        // c. Behind contact midpoint
        if let Some(behind_contact) = closest_behind_contact {
            let behind_pos = behind_contact.position_at(current_t + 1);
            let behind_d = next_our_pos.distance(behind_pos);
            let mid_d = (d + behind_d) / 2.0;
            candidates.push((
                RadarJob {
                    angle: initial_job.angle,
                    width: initial_job.width,
                    min_distance: initial_job.min_distance,
                    max_distance: mid_d,
                    state: RadarState::Tracking {
                        contact_id: contact.id,
                    },
                },
                "Behind Mid".to_string(),
            ));
        }

        // d. In front contact midpoint
        if let Some(front_contact) = closest_front_contact {
            let front_pos = front_contact.position_at(current_t + 1);
            let front_d = next_our_pos.distance(front_pos);
            let mid_d = (d + front_d) / 2.0;
            candidates.push((
                RadarJob {
                    angle: initial_job.angle,
                    width: initial_job.width,
                    min_distance: mid_d,
                    max_distance: initial_job.max_distance,
                    state: RadarState::Tracking {
                        contact_id: contact.id,
                    },
                },
                "Front Mid".to_string(),
            ));
        }

        for (job, _) in &mut candidates {
            constrain_job(job);
        }

        for (job, label) in &candidates {
            let draw_label = format!("Cand {} (cid {})", label, contact.id);
            draw_job_beam(job, rgb(128, 128, 255), &draw_label);
        }

        let contacts_match_by_beam = |job: &RadarJob| {
            nearby_contacts
                .iter()
                .filter(|other| self.is_contact_matched_by_beam(other, job, current_t))
                .count()
        };

        let contact_score = |b: &RadarJob| {
            (
                contacts_match_by_beam(b),
                b.width.to_bits(),
                (b.max_distance - b.min_distance).to_bits(),
            )
        };
        let chosen = candidates
            .iter()
            .min_by_key(|&(b, _)| contact_score(b))
            .cloned();
        if let Some((ref job, ref label)) = chosen {
            draw_job_beam(
                job,
                rgb(0, 255, 0),
                &format!("Selected: {} (cid {})", label, contact.id),
            );
        }
        chosen.map(|(job, _)| job)
    }

    pub fn update_target(&mut self, our_pos: Vec2, our_vel: Vec2) -> Option<Contact> {
        if let Some(target_id) = self.current_target_id {
            if !self
                .contacts
                .iter()
                .any(|c| c.id == target_id && !c.provisional)
            {
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
                let target_pos_f =
                    contact.position_at(current_tick() + (t_horizon / TICK_LENGTH).round() as u32);
                let our_pos_f = our_pos + our_vel * t_horizon;
                let future_dist = target_pos_f.distance(our_pos_f);

                if future_dist < min_future_dist {
                    min_future_dist = future_dist;
                    closest_id = Some(contact.id);
                }
            }
            self.current_target_id = closest_id;
        }

        self.current_target_id
            .and_then(|id| self.contacts.iter().find(|c| c.id == id).cloned())
    }
}

#[cfg(test)]
mod radar_test;
