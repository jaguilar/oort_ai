use oort_api::prelude::*;

#[derive(Clone, Debug)]
pub struct Contact {
    pub id: u32,
    pub class: Class,
    pub position: Vec2,
    pub velocity: Vec2,
    pub acceleration: Vec2,
    pub last_scanned: u32,
    pub rssi: f64,
    pub snr: f64,
    pub pos_uncertainty: f64,
    pub vel_uncertainty: f64,
}

#[derive(Clone, Copy, PartialEq)]
pub enum SplitPhase {
    Front,
    Behind,
}

#[derive(Clone, Copy, PartialEq)]
pub enum RadarState {
    Scanning { ticks_left: u32 },
    ScanningSplit {
        ticks_left: u32,
        sweep_head: f64,
        contact_distance: f64,
        phase: SplitPhase,
    },
    Tracking { contact_id: u32 },
}

pub struct RadarController {
    contacts: Vec<Contact>,
    next_contact_id: u32,
    active_radar_state: RadarState,
    current_target_id: Option<u32>,
    search_width: f64,
    tracking_width: f64,
    max_distance: f64,
    gate_radius: f64,
}

impl RadarController {
    pub fn new() -> Self {
        Self {
            contacts: Vec::new(),
            next_contact_id: 0,
            active_radar_state: RadarState::Scanning { ticks_left: 11 },
            current_target_id: None,
            search_width: 0.6,
            tracking_width: 0.05,
            max_distance: 10000.0,
            gate_radius: 200.0,
        }
    }

    pub fn set_search_width(&mut self, width: f64) {
        self.search_width = width;
    }

    pub fn set_tracking_width(&mut self, width: f64) {
        self.tracking_width = width;
    }

    pub fn set_max_distance(&mut self, dist: f64) {
        self.max_distance = dist;
    }

    pub fn set_gate_radius(&mut self, radius: f64) {
        self.gate_radius = radius;
    }

    pub fn contacts(&self) -> &[Contact] {
        &self.contacts
    }

    pub fn geometric_center(&self, default_pos: Vec2) -> Vec2 {
        if self.contacts.is_empty() {
            default_pos
        } else {
            let sum = self.contacts.iter().fold(Vec2::new(0.0, 0.0), |acc, c| acc + c.position);
            sum / self.contacts.len() as f64
        }
    }

    pub fn update(&mut self) {
        select_radar(0);
        let scan_result = scan();
        let current_t = current_tick();

        // 1. Process scan result depending on active_radar_state
        if let Some(ref c) = scan_result {
            match self.active_radar_state {
                RadarState::Scanning { .. } | RadarState::ScanningSplit { .. } => {
                    let mut best_match: Option<&mut Contact> = None;
                    let mut best_dist = f64::MAX;

                    for contact in &mut self.contacts {
                        if contact.class == c.class {
                            let dist = contact.position.distance(c.position);
                            if dist < (3.89 * contact.pos_uncertainty).max(250.0) && dist < best_dist {
                                best_dist = dist;
                                best_match = Some(contact);
                            }
                        }
                    }

                    if let Some(contact) = best_match {
                        let dt = (current_t - contact.last_scanned) as f64 * TICK_LENGTH;
                        if dt > 0.0 {
                            let raw_accel = (c.velocity - contact.velocity) / dt;
                            let alpha = 0.5;
                            contact.acceleration = alpha * raw_accel + (1.0 - alpha) * contact.acceleration;
                        }
                        contact.position = c.position;
                        contact.velocity = c.velocity;
                        contact.last_scanned = current_t;
                        contact.rssi = c.rssi;
                        contact.snr = c.snr;
                        let error_factor = 10.0f64.powf(-c.snr / 10.0);
                        let dist = position().distance(c.position);
                        let sigma_r = 10000.0 * error_factor;
                        let sigma_theta = (10.0 * (TAU / 360.0)) * error_factor;
                        contact.pos_uncertainty = sigma_r.max(dist * sigma_theta.min(radar_width() / 2.0));
                        contact.vel_uncertainty = 100.0 * error_factor;
                    } else {
                        let error_factor = 10.0f64.powf(-c.snr / 10.0);
                        let dist = position().distance(c.position);
                        let sigma_r = 10000.0 * error_factor;
                        let sigma_theta = (10.0 * (TAU / 360.0)) * error_factor;
                        let pos_unc = sigma_r.max(dist * sigma_theta.min(radar_width() / 2.0));
                        let vel_unc = 100.0 * error_factor;
                        self.contacts.push(Contact {
                            id: self.next_contact_id,
                            class: c.class,
                            position: c.position,
                            velocity: c.velocity,
                            acceleration: Vec2::new(0.0, 0.0),
                            last_scanned: current_t,
                            rssi: c.rssi,
                            snr: c.snr,
                            pos_uncertainty: pos_unc,
                            vel_uncertainty: vel_unc,
                        });
                        self.next_contact_id += 1;
                    }
                }
                RadarState::Tracking { contact_id } => {
                    let mut found = false;
                    if let Some(contact) = self.contacts.iter_mut().find(|co| co.id == contact_id) {
                        if contact.class == c.class && contact.position.distance(c.position) < (3.89 * contact.pos_uncertainty).max(250.0) {
                            found = true;
                            let dt = (current_t - contact.last_scanned) as f64 * TICK_LENGTH;
                            if dt > 0.0 {
                                let raw_accel = (c.velocity - contact.velocity) / dt;
                                let alpha = 0.5;
                                contact.acceleration = alpha * raw_accel + (1.0 - alpha) * contact.acceleration;
                            }
                            contact.position = c.position;
                            contact.velocity = c.velocity;
                            contact.last_scanned = current_t;
                            contact.rssi = c.rssi;
                            contact.snr = c.snr;
                            let error_factor = 10.0f64.powf(-c.snr / 10.0);
                            let dist = position().distance(c.position);
                            let sigma_r = 10000.0 * error_factor;
                            let sigma_theta = (10.0 * (TAU / 360.0)) * error_factor;
                            contact.pos_uncertainty = sigma_r.max(dist * sigma_theta.min(radar_width() / 2.0));
                            contact.vel_uncertainty = 100.0 * error_factor;
                        }
                    }
                    if !found {
                        self.contacts.retain(|co| co.id != contact_id);
                    }
                }
            }
        } else {
            if let RadarState::Tracking { contact_id } = self.active_radar_state {
                self.contacts.retain(|co| co.id != contact_id);
            }
        }

        // 2. Tracking update: extrapolate positions for contacts not scanned in this tick
        for contact in &mut self.contacts {
            if contact.last_scanned < current_t {
                let dt = TICK_LENGTH;
                contact.position += contact.velocity * dt + 0.5 * contact.acceleration * dt * dt;
                contact.velocity += contact.acceleration * dt;
                contact.pos_uncertainty += contact.vel_uncertainty * dt;
            }
        }

        // 3. Prune old contacts (timeout after 120 ticks / 2 seconds)
        self.contacts.retain(|c| current_t - c.last_scanned <= 120);

        // 4. Schedule radar state and configure hardware for NEXT tick
        let next_radar_state = match self.active_radar_state {
            RadarState::Scanning { ticks_left } => {
                let next_ticks = if ticks_left > 1 { ticks_left - 1 } else { 0 };
                if next_ticks > 0 {
                    let sweep_head = radar_heading() + self.search_width;
                    let mut intersecting: Option<f64> = None;
                    for contact in &self.contacts {
                        let angle = (contact.position - position()).angle();
                        if angle_diff(sweep_head, angle).abs() < self.search_width / 2.0 {
                            intersecting = Some(position().distance(contact.position));
                            break;
                        }
                    }
                    if let Some(dist) = intersecting {
                        RadarState::ScanningSplit {
                            ticks_left: next_ticks,
                            sweep_head,
                            contact_distance: dist,
                            phase: SplitPhase::Front,
                        }
                    } else {
                        RadarState::Scanning { ticks_left: next_ticks }
                    }
                } else {
                    if let Some(first_contact) = self.contacts.iter().min_by_key(|c| c.id) {
                        RadarState::Tracking { contact_id: first_contact.id }
                    } else {
                        let sweep_head = radar_heading() + self.search_width;
                        let mut intersecting: Option<f64> = None;
                        for contact in &self.contacts {
                            let angle = (contact.position - position()).angle();
                            if angle_diff(sweep_head, angle).abs() < self.search_width / 2.0 {
                                intersecting = Some(position().distance(contact.position));
                                break;
                            }
                        }
                        if let Some(dist) = intersecting {
                            RadarState::ScanningSplit {
                                ticks_left: 11,
                                sweep_head,
                                contact_distance: dist,
                                phase: SplitPhase::Front,
                            }
                        } else {
                            RadarState::Scanning { ticks_left: 11 }
                        }
                    }
                }
            }
            RadarState::ScanningSplit { ticks_left, sweep_head, contact_distance, phase } => {
                match phase {
                    SplitPhase::Front => {
                        RadarState::ScanningSplit {
                            ticks_left,
                            sweep_head,
                            contact_distance,
                            phase: SplitPhase::Behind,
                        }
                    }
                    SplitPhase::Behind => {
                        if ticks_left > 0 {
                            let next_sweep = sweep_head + self.search_width;
                            let mut intersecting: Option<f64> = None;
                            for contact in &self.contacts {
                                let angle = (contact.position - position()).angle();
                                if angle_diff(next_sweep, angle).abs() < self.search_width / 2.0 {
                                    intersecting = Some(position().distance(contact.position));
                                    break;
                                }
                            }
                            if let Some(dist) = intersecting {
                                RadarState::ScanningSplit {
                                    ticks_left,
                                    sweep_head: next_sweep,
                                    contact_distance: dist,
                                    phase: SplitPhase::Front,
                                }
                            } else {
                                RadarState::Scanning { ticks_left }
                            }
                        } else {
                            if let Some(first_contact) = self.contacts.iter().min_by_key(|c| c.id) {
                                RadarState::Tracking { contact_id: first_contact.id }
                            } else {
                                let next_sweep = sweep_head + self.search_width;
                                let mut intersecting: Option<f64> = None;
                                for contact in &self.contacts {
                                    let angle = (contact.position - position()).angle();
                                    if angle_diff(next_sweep, angle).abs() < self.search_width / 2.0 {
                                        intersecting = Some(position().distance(contact.position));
                                        break;
                                    }
                                }
                                if let Some(dist) = intersecting {
                                    RadarState::ScanningSplit {
                                        ticks_left: 11,
                                        sweep_head: next_sweep,
                                        contact_distance: dist,
                                        phase: SplitPhase::Front,
                                    }
                                } else {
                                    RadarState::Scanning { ticks_left: 11 }
                                }
                            }
                        }
                    }
                }
            }
            RadarState::Tracking { contact_id } => {
                let next_contact = self.contacts.iter()
                    .filter(|c| c.id > contact_id)
                    .min_by_key(|c| c.id);
                if let Some(contact) = next_contact {
                    RadarState::Tracking { contact_id: contact.id }
                } else {
                    let sweep_head = radar_heading() + self.search_width;
                    let mut intersecting: Option<f64> = None;
                    for contact in &self.contacts {
                        let angle = (contact.position - position()).angle();
                        if angle_diff(sweep_head, angle).abs() < self.search_width / 2.0 {
                            intersecting = Some(position().distance(contact.position));
                            break;
                        }
                    }
                    if let Some(dist) = intersecting {
                        RadarState::ScanningSplit {
                            ticks_left: 11,
                            sweep_head,
                            contact_distance: dist,
                            phase: SplitPhase::Front,
                        }
                    } else {
                        RadarState::Scanning { ticks_left: 11 }
                    }
                }
            }
        };

        // Configure hardware for next tick based on next_radar_state
        match next_radar_state {
            RadarState::Scanning { .. } => {
                let sweep_head = radar_heading() + self.search_width;
                set_radar_heading(sweep_head);
                set_radar_width(self.search_width);
                set_radar_min_distance(0.0);
                set_radar_max_distance(self.max_distance);
            }
            RadarState::ScanningSplit { sweep_head, contact_distance, phase, .. } => {
                set_radar_heading(sweep_head);
                set_radar_width(self.search_width);
                match phase {
                    SplitPhase::Front => {
                        set_radar_min_distance(0.0);
                        set_radar_max_distance((contact_distance - 150.0).max(0.0));
                    }
                    SplitPhase::Behind => {
                        set_radar_min_distance(contact_distance + 150.0);
                        set_radar_max_distance(self.max_distance);
                    }
                }
            }
            RadarState::Tracking { contact_id } => {
                if let Some(contact) = self.contacts.iter().find(|c| c.id == contact_id) {
                    let next_pos = contact.position + contact.velocity * TICK_LENGTH + 0.5 * contact.acceleration * TICK_LENGTH * TICK_LENGTH;
                    let next_our_pos = position() + velocity() * TICK_LENGTH;
                    let d = next_our_pos.distance(next_pos);
                    let angle = (next_pos - next_our_pos).angle();
                    set_radar_heading(angle);
                    
                    // Extrapolate position uncertainty to the next tick's scan
                    let next_pos_uncertainty = contact.pos_uncertainty + contact.vel_uncertainty * TICK_LENGTH;
                    let gate_radius = (3.89 * next_pos_uncertainty).max(self.gate_radius);
                    
                    // Dynamic tracking beam width based on distance and dynamic gate radius
                    let calculated_width = (2.0 * gate_radius / d).clamp(0.005, self.tracking_width);
                    set_radar_width(calculated_width);
                    
                    // Distance clipping to isolate the target from noise/jamming
                    set_radar_min_distance((d - gate_radius).max(0.0));
                    set_radar_max_distance(d + gate_radius);
                } else {
                    let sweep_head = radar_heading() + self.search_width;
                    set_radar_heading(sweep_head);
                    set_radar_width(self.search_width);
                    set_radar_min_distance(0.0);
                    set_radar_max_distance(self.max_distance);
                }
            }
        }

        // 5. Draw 99.99% confidence intervals for all active contacts
        for contact in &self.contacts {
            let radius = 3.89 * contact.pos_uncertainty;
            draw_polygon(contact.position, radius, 16, 0.0, rgb(255, 165, 0)); // Orange color
            
            // Draw a label with range and uncertainty
            let text_pos = contact.position + vec2(0.0, radius + 20.0);
            draw_text!(text_pos, rgb(255, 165, 0), "Target CI: {:.1}m", radius);
        }

        self.active_radar_state = next_radar_state;
    }

    pub fn update_target(&mut self, our_pos: Vec2, our_vel: Vec2) -> Option<Contact> {
        if let Some(target_id) = self.current_target_id {
            if !self.contacts.iter().any(|c| c.id == target_id) {
                self.current_target_id = None;
            }
        }

        if self.current_target_id.is_none() {
            let mut closest_id = None;
            let mut min_future_dist = f64::MAX;
            for contact in &self.contacts {
                let t_horizon = 2.0;
                let target_pos_f = contact.position + contact.velocity * t_horizon + 0.5 * contact.acceleration * t_horizon * t_horizon;
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
