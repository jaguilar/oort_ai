//! Compilable examples for the Oort Radar and Targeting APIs.
//! Covers selecting radars, heading and sweep adjustments, contact filters, scanning, and jamming.

use oort_api::prelude::*;

/// Example of configuring and querying the active radar systems.
pub fn show_radar_configuration() {
    // 1. Select the radar to command (ships have 2 independent radar systems: 0 and 1)
    let radar_index = 0;
    select_radar(radar_index);

    // 2. Adjust the radar's heading (direction of the main beam, in world radians)
    let desired_direction = 0.5;
    set_radar_heading(desired_direction);
    let current_heading: f64 = radar_heading();

    // 3. Set the radar sweep width / field of view (in radians)
    // A wider sweep covers more area but reduces signal quality or search speed
    let fov_radians = 0.2; // roughly 11.5 degrees
    set_radar_width(fov_radians);
    let current_width: f64 = radar_width();

    // 4. Configure distance filters (in meters) to focus scan and reject clutter
    let min_dist = 100.0;
    let max_dist = 5000.0;
    set_radar_min_distance(min_dist);
    set_radar_max_distance(max_dist);

    let active_min = radar_min_distance();
    let active_max = radar_max_distance();

    debug!("Radar {}: heading: {}, width: {}, range: [{}m, {}m]", 
        radar_index, current_heading, current_width, active_min, active_max);
}

/// Example of scanning for contacts and processing the scan result.
pub fn show_radar_scanning() {
    // Select radar 0 and sweep it dynamically
    select_radar(0);
    
    // Perform a scan for the current tick
    // It returns the radar contact with the highest signal strength in the scan sector
    let contact: Option<ScanResult> = scan();

    match contact {
        Some(result) => {
            let enemy_class: Class = result.class;
            let enemy_pos: Vec2 = result.position;
            let enemy_vel: Vec2 = result.velocity;
            
            // Received Signal Strength Indicator (dBm)
            let signal_strength: f64 = result.rssi;
            
            // Signal-to-Noise Ratio (dB)
            let snr_value: f64 = result.snr;

            debug!("Contact found! Class: {:?}", enemy_class);
            debug!("Position: {:?}, Velocity: {:?}", enemy_pos, enemy_vel);
            debug!("RSSI: {} dBm, SNR: {} dB", signal_strength, snr_value);

            // Once a contact is found, narrow the beam to track it tightly
            set_radar_width(0.01);
            
            // Point the radar directly at the contact's estimated position
            let to_contact = enemy_pos - position();
            set_radar_heading(to_contact.angle());
        }
        None => {
            debug!("No contacts in sweep.");
            // Keep sweeping by panning the radar beam
            let new_heading = radar_heading() + 0.1;
            set_radar_heading(new_heading);
            set_radar_width(0.5); // Open up wide to search again
        }
    }
}

/// Example of activating/querying Electronic Countermeasures (ECM) on the radar.
pub fn show_radar_ecm() {
    select_radar(0);

    // 1. Check current ECM mode
    let current_ecm: EcmMode = radar_ecm_mode();
    debug!("Current radar ECM Mode: {:?}", current_ecm);

    // 2. Set ECM mode to Noise jamming
    // In EcmMode::Noise, affected enemy radars will experience a reduced SNR,
    // making it harder for them to detect/track us.
    set_radar_ecm_mode(EcmMode::Noise);
    
    // 3. We can disable ECM by switching back to None
    set_radar_ecm_mode(EcmMode::None);
}

/// Example of using tutorial target helper functions (only valid in training scenarios).
pub fn show_tutorial_helpers() {
    // Retrieves the absolute position of the target set by the scenario
    let target_pos: Vec2 = target();
    
    // Retrieves the absolute velocity of the target
    let target_vel: Vec2 = target_velocity();

    let distance_to_target = position().distance(target_pos);
    
    debug!("Tutorial Target: Pos: {:?}, Vel: {:?}, Dist: {}m", 
        target_pos, target_vel, distance_to_target);
}
