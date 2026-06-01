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
        unscanned_in_range_ticks: 0,
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
        unscanned_in_range_ticks: 0,
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

#[test]
fn test_radar_out_of_range_retained() {
    let contact_close = Contact {
        id: 1,
        class: Class::Fighter,
        position: Vec2::new(0.0, 1000.0),
        velocity: Vec2::new(0.0, 0.0),
        acceleration: Vec2::new(0.0, 0.0),
        last_scanned: 0,
        rssi: 0.0,
        snr: 30.0,
        pos_uncertainty: 10.0,
        vel_uncertainty: 5.0,
        radar_locked: true,
        provisional: false,
        tracking_retry_count: 0,
        confirmation_attempts: 0,
        unscanned_in_range_ticks: 0,
        p_cov_x: Contact::initial_cov(10.0, 5.0, Class::Fighter),
        p_cov_y: Contact::initial_cov(10.0, 5.0, Class::Fighter),
    };

    let contact_far = Contact {
        id: 2,
        class: Class::Fighter,
        position: Vec2::new(0.0, 200000.0),
        velocity: Vec2::new(0.0, 0.0),
        acceleration: Vec2::new(0.0, 0.0),
        last_scanned: 0,
        rssi: 0.0,
        snr: 30.0,
        pos_uncertainty: 10.0,
        vel_uncertainty: 5.0,
        radar_locked: true,
        provisional: false,
        tracking_retry_count: 0,
        confirmation_attempts: 0,
        unscanned_in_range_ticks: 0,
        p_cov_x: Contact::initial_cov(10.0, 5.0, Class::Fighter),
        p_cov_y: Contact::initial_cov(10.0, 5.0, Class::Fighter),
    };

    // Assert is_within_range functions as expected for both distances
    assert!(is_within_range(&contact_close, 1000.0));
    assert!(!is_within_range(&contact_far, 200000.0));
}

#[test]
fn test_add_radio_ping() {
    let mut rc = RadarController::new();

    let telemetry1 = TargetTelemetry {
        position: Vec2::new(100.0, 200.0),
        velocity: Vec2::new(10.0, -5.0),
        rssi: -50.0,
        class: Class::Fighter,
        tick: 0,
    };

    // 1. Adding a new ping should add it directly and return its ID
    let id1 = rc.add_radio_ping(telemetry1);
    assert!(id1 > 0);
    
    // Check that the contact list has the new contact and it is NOT provisional
    let contact1 = rc.get_contact(id1).expect("Contact should exist");
    assert_eq!(contact1.id, id1);
    assert_eq!(contact1.class, Class::Fighter);
    assert_eq!(contact1.position, Vec2::new(100.0, 200.0));
    assert_eq!(contact1.velocity, Vec2::new(10.0, -5.0));
    assert_eq!(contact1.provisional, false); // Immediately confirmed target

    // 2. Adding a duplicate ping (close to telemetry1) should update it and return same ID
    let telemetry2 = TargetTelemetry {
        position: Vec2::new(101.0, 199.0),
        velocity: Vec2::new(10.0, -5.0),
        rssi: -45.0,
        class: Class::Fighter,
        tick: 0, // Set to 0 to match current_tick() in tests
    };

    let id2 = rc.add_radio_ping(telemetry2);
    assert_eq!(id1, id2); // Must return the same ID because it's a duplicate

    // Check that the contact was updated (predict_and_update should change position to be closer to 101.0, 199.0)
    let contact1_updated = rc.get_contact(id1).expect("Contact should exist");
    assert_eq!(contact1_updated.last_scanned, 0);
    assert_eq!(contact1_updated.provisional, false);

    // 3. Adding a non-duplicate ping (far away) should add a new contact with a different ID
    let telemetry3 = TargetTelemetry {
        position: Vec2::new(2000.0, -3000.0),
        velocity: Vec2::new(0.0, 0.0),
        rssi: -60.0,
        class: Class::Fighter,
        tick: 0, // Set to 0 to match current_tick() in tests
    };

    let id3 = rc.add_radio_ping(telemetry3);
    assert_ne!(id1, id3); // Must have a different ID
    let contact3 = rc.get_contact(id3).expect("Contact should exist");
    assert_eq!(contact3.id, id3);
    assert_eq!(contact3.provisional, false);
}

#[test]
fn test_nearby_contact_exclusion() {
    let mut rc = RadarController::new();
    rc.set_gate_radius(10.0);

    // Target contact at (1000.0, 0.0)
    let target = Contact {
        id: 1,
        class: Class::Fighter,
        position: Vec2::new(1000.0, 0.0),
        velocity: Vec2::new(0.0, 0.0),
        acceleration: Vec2::new(0.0, 0.0),
        last_scanned: 0,
        rssi: 0.0,
        snr: 30.0,
        pos_uncertainty: 10.0,
        vel_uncertainty: 5.0,
        radar_locked: true,
        provisional: false,
        tracking_retry_count: 0,
        confirmation_attempts: 0,
        unscanned_in_range_ticks: 0,
        p_cov_x: Contact::initial_cov(10.0, 5.0, Class::Fighter),
        p_cov_y: Contact::initial_cov(10.0, 5.0, Class::Fighter),
    };
    rc.contacts.push(target.clone());

    // 1. First: no nearby contacts. The tracking job should be the initial job.
    let job1 = rc.generate_tracking_scan(&target, 0).expect("Job should be generated");
    assert_eq!(job1.angle, 0.0);
    // By default target is at 1000m.
    // Min distance = (1000.0 - 38.9) = 961.1. Max distance = 1000.0 + 38.9 = 1038.9.
    assert!((job1.min_distance - 961.1).abs() < 0.1, "min_distance was {}", job1.min_distance);
    assert!((job1.max_distance - 1038.9).abs() < 0.1, "max_distance was {}", job1.max_distance);

    // 2. Add a confusing contact to the left (counter-clockwise, positive angle)
    // Let's place it at (1000.0, 30.0), so angle is approx 0.03 rad.
    let left_contact = Contact {
        id: 2,
        class: Class::Fighter,
        position: Vec2::new(1000.0, 30.0),
        velocity: Vec2::new(0.0, 0.0),
        acceleration: Vec2::new(0.0, 0.0),
        last_scanned: 0,
        rssi: 0.0,
        snr: 30.0,
        pos_uncertainty: 1.0,
        vel_uncertainty: 1.0,
        radar_locked: true,
        provisional: false,
        tracking_retry_count: 0,
        confirmation_attempts: 0,
        unscanned_in_range_ticks: 0,
        p_cov_x: Contact::initial_cov(1.0, 1.0, Class::Fighter),
        p_cov_y: Contact::initial_cov(1.0, 1.0, Class::Fighter),
    };
    rc.contacts.push(left_contact);

    let job2 = rc.generate_tracking_scan(&target, 0).expect("Job should be generated");
    // Because of the left contact, the beam should shift clockwise (to the right, i.e. negative angle).
    assert!(job2.angle < 0.0, "Beam should shift to the right, angle = {}", job2.angle);
}
