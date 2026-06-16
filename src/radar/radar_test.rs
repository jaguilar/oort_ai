use super::*;

#[test]
fn test_kalman_filter() {
    let mut contact = Contact {
        id: 0,
        kinematic: KinematicState::new(Class::Fighter, Vec2::new(0.0, 0.0), Vec2::new(10.0, 0.0), Vec2::new(0.0, 0.0), 0),
        last_measurement_tick: 0,
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
        prioritize_scan: false,
        prev_scan_pos_uncertainty: None,
        low_improvement_consecutive_scans: 0,
        last_beam_width: None,
        last_beam_center: None,
        last_beam_center_pos: None,
        missile_scan_ticks_remaining: 0,
        scan_boundary_points: None,
        scan_boundary_vels: None,
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

        contact.predict(t);
        contact.update_with_measurement(meas_pos, meas_vel, sigma_p, sigma_v, t);

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
        kinematic: KinematicState::new(Class::Fighter, Vec2::new(0.0, 0.0), Vec2::new(0.0, 0.0), Vec2::new(0.0, 0.0), 0),
        last_measurement_tick: 0,
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
        prioritize_scan: false,
        prev_scan_pos_uncertainty: None,
        low_improvement_consecutive_scans: 0,
        last_beam_width: None,
        last_beam_center: None,
        last_beam_center_pos: None,
        missile_scan_ticks_remaining: 0,
        scan_boundary_points: None,
        scan_boundary_vels: None,
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
        kinematic: KinematicState::new(Class::Fighter, Vec2::new(0.0, 1000.0), Vec2::new(0.0, 0.0), Vec2::new(0.0, 0.0), 0),
        last_measurement_tick: 0,
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
        prioritize_scan: false,
        prev_scan_pos_uncertainty: None,
        low_improvement_consecutive_scans: 0,
        last_beam_width: None,
        last_beam_center: None,
        last_beam_center_pos: None,
        missile_scan_ticks_remaining: 0,
        scan_boundary_points: None,
        scan_boundary_vels: None,
    };

    let contact_far = Contact {
        id: 2,
        kinematic: KinematicState::new(Class::Fighter, Vec2::new(0.0, 200000.0), Vec2::new(0.0, 0.0), Vec2::new(0.0, 0.0), 0),
        last_measurement_tick: 0,
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
        prioritize_scan: false,
        prev_scan_pos_uncertainty: None,
        low_improvement_consecutive_scans: 0,
        last_beam_width: None,
        last_beam_center: None,
        last_beam_center_pos: None,
        missile_scan_ticks_remaining: 0,
        scan_boundary_points: None,
        scan_boundary_vels: None,
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
        kinematic: KinematicState::new(Class::Fighter, Vec2::new(1000.0, 0.0), Vec2::new(0.0, 0.0), Vec2::new(0.0, 0.0), 0),
        last_measurement_tick: 0,
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
        prioritize_scan: false,
        prev_scan_pos_uncertainty: None,
        low_improvement_consecutive_scans: 0,
        last_beam_width: None,
        last_beam_center: None,
        last_beam_center_pos: None,
        missile_scan_ticks_remaining: 0,
        scan_boundary_points: None,
        scan_boundary_vels: None,
    };
    rc.contacts.push(target.clone());

    // 1. First: no nearby contacts. The tracking job should be the initial job.
    let job1 = rc.generate_tracking_scan(&target, 0).expect("Job should be generated");
    assert_eq!(job1.angle, 0.0);
    // By default target is at 1000m.
    // Min distance = (1000.0 - 32.9) = 967.1. Max distance = 1000.0 + 32.9 = 1032.9.
    assert!((job1.min_distance - 967.1).abs() < 0.1, "min_distance was {}", job1.min_distance);
    assert!((job1.max_distance - 1032.9).abs() < 0.1, "max_distance was {}", job1.max_distance);

    // 2. Add a confusing contact to the left (counter-clockwise, positive angle)
    // Let's place it at (1000.0, 30.0), so angle is approx 0.03 rad.
    let left_contact = Contact {
        id: 2,
        kinematic: KinematicState::new(Class::Fighter, Vec2::new(1000.0, 30.0), Vec2::new(0.0, 0.0), Vec2::new(0.0, 0.0), 0),
        last_measurement_tick: 0,
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
        prioritize_scan: false,
        prev_scan_pos_uncertainty: None,
        low_improvement_consecutive_scans: 0,
        last_beam_width: None,
        last_beam_center: None,
        last_beam_center_pos: None,
        missile_scan_ticks_remaining: 0,
        scan_boundary_points: None,
        scan_boundary_vels: None,
    };
    rc.contacts.push(left_contact);

    let job2 = rc.generate_tracking_scan(&target, 0).expect("Job should be generated");
    // Because of the left contact, the beam should shift clockwise (to the right, i.e. negative angle).
    assert!(job2.angle < 0.0, "Beam should shift to the right, angle = {}", job2.angle);
}

#[test]
fn test_radar_pings_reliable_range() {
    let mut rc = RadarController::new();

    // Max range calculations for Fighter in tests (power=100e3, rx_xs=10.0, rcs=10.0):
    // reliable_rssi = 1e-12
    // If slice_width = 0.6: max_range ~ 40327m
    // If slice_width = 0.05: max_range ~ 75056m

    // A detection at 50000m (outside reliable range at 0.6, inside at 0.05)
    let scan_hit = ScanResult {
        position: Vec2::new(50000.0, 0.0),
        velocity: Vec2::new(0.0, 0.0),
        class: Class::Fighter,
        rssi: 1e-11,
        snr: 25.0,
    };

    // 1. Scan with slice_width = 0.6: should NOT add a new contact (returns 0, contacts.len() is 0)
    let id1 = rc.process_scan_hit(scan_hit.clone(), Some(ScanSlice {
        angle: 0.0,
        width: 0.6,
        min_distance: 0.0,
        max_distance: 100000.0,
    }));
    assert_eq!(id1, 0);
    assert!(rc.contacts.is_empty());

    // 2. Scan with slice_width = 0.05: should successfully add a new contact (returns id > 0, contacts.len() is 1)
    let id2 = rc.process_scan_hit(scan_hit.clone(), Some(ScanSlice {
        angle: 0.0,
        width: 0.05,
        min_distance: 0.0,
        max_distance: 100000.0,
    }));
    assert!(id2 > 0);
    assert_eq!(rc.contacts.len(), 1);
    assert_eq!(rc.contacts[0].id, id2);

    // 3. Scan with slice_width = None (e.g. radio ping): should successfully add a new contact even if far away
    let mut rc2 = RadarController::new();
    let id3 = rc2.process_scan_hit(scan_hit.clone(), None);
    assert!(id3 > 0);
    assert_eq!(rc2.contacts.len(), 1);
}

#[test]
fn test_radar_prunes_moving_away_missiles_and_torpedoes() {
    let mut rc = RadarController::new();

    // 1. Incoming missile: pos = (100, 0), vel = (-10, 0)
    let m_incoming = ScanResult {
        position: Vec2::new(100.0, 0.0),
        velocity: Vec2::new(-10.0, 0.0),
        class: Class::Missile,
        rssi: -50.0,
        snr: 25.0,
    };

    // 2. Moving-away missile: pos = (500, 0), vel = (10, 0)
    let m_away = ScanResult {
        position: Vec2::new(500.0, 0.0),
        velocity: Vec2::new(10.0, 0.0),
        class: Class::Missile,
        rssi: -50.0,
        snr: 25.0,
    };

    // 3. Incoming torpedo: pos = (200, 0), vel = (-5, 0)
    let t_incoming = ScanResult {
        position: Vec2::new(200.0, 0.0),
        velocity: Vec2::new(-5.0, 0.0),
        class: Class::Torpedo,
        rssi: -50.0,
        snr: 25.0,
    };

    // 4. Moving-away torpedo: pos = (600, 0), vel = (5, 0)
    let t_away = ScanResult {
        position: Vec2::new(600.0, 0.0),
        velocity: Vec2::new(5.0, 0.0),
        class: Class::Torpedo,
        rssi: -50.0,
        snr: 25.0,
    };

    // 5. Moving-away fighter (should NOT be pruned): pos = (300, 0), vel = (20, 0)
    let f_away = ScanResult {
        position: Vec2::new(300.0, 0.0),
        velocity: Vec2::new(20.0, 0.0),
        class: Class::Fighter,
        rssi: -50.0,
        snr: 25.0,
    };

    // Add them to the radar controller using process_scan_hit
    let id_m_in = rc.process_scan_hit(m_incoming, None);
    let id_m_away = rc.process_scan_hit(m_away, None);
    let id_t_in = rc.process_scan_hit(t_incoming, None);
    let id_t_away = rc.process_scan_hit(t_away, None);
    let id_f_away = rc.process_scan_hit(f_away, None);

    // Verify which ones are kept
    assert!(rc.has_contact(id_m_in));
    assert!(!rc.has_contact(id_m_away));
    assert!(rc.has_contact(id_t_in));
    assert!(!rc.has_contact(id_t_away));
    assert!(rc.has_contact(id_f_away));
}

#[test]
fn test_missile_priority_scanning_initialization() {
    let mut rc = RadarController::new();

    let m_incoming = ScanResult {
        position: Vec2::new(100.0, 0.0),
        velocity: Vec2::new(-10.0, 0.0),
        class: Class::Missile,
        rssi: -50.0,
        snr: 25.0,
    };

    let id = rc.process_scan_hit(m_incoming, None);
    assert!(id > 0);
    
    let contact = rc.get_contact(id).expect("Contact should exist");
    assert_eq!(contact.class, Class::Missile);
    assert_eq!(contact.prioritize_scan, true);
    assert!(contact.prev_scan_pos_uncertainty.is_some());

    let f_incoming = ScanResult {
        position: Vec2::new(200.0, 0.0),
        velocity: Vec2::new(-10.0, 0.0),
        class: Class::Fighter,
        rssi: -50.0,
        snr: 25.0,
    };

    let id2 = rc.process_scan_hit(f_incoming, None);
    assert!(id2 > 0);
    let contact2 = rc.get_contact(id2).expect("Contact should exist");
    assert_eq!(contact2.class, Class::Fighter);
    assert_eq!(contact2.prioritize_scan, false);
}

#[test]
fn test_jamming_mode() {
    let mut rc = RadarController::new();
    rc.jamming_mode = true;
    rc.priority_targets = vec![42];

    let contact = Contact {
        id: 42,
        kinematic: KinematicState::new(Class::Fighter, Vec2::new(1000.0, 0.0), Vec2::new(0.0, 0.0), Vec2::new(0.0, 0.0), current_tick()),
        last_measurement_tick: current_tick(),
        rssi: -50.0,
        snr: 40.0,
        pos_uncertainty: 10.0,
        vel_uncertainty: 5.0,
        radar_locked: true,
        provisional: false,
        tracking_retry_count: 0,
        confirmation_attempts: 0,
        unscanned_in_range_ticks: 0,
        p_cov_x: Contact::initial_cov(10.0, 5.0, Class::Fighter),
        p_cov_y: Contact::initial_cov(10.0, 5.0, Class::Fighter),
        prioritize_scan: false,
        prev_scan_pos_uncertainty: None,
        low_improvement_consecutive_scans: 0,
        last_beam_width: None,
        last_beam_center: None,
        last_beam_center_pos: None,
        missile_scan_ticks_remaining: 0,
        scan_boundary_points: None,
        scan_boundary_vels: None,
    };
    rc.contacts.push(contact);

    // 1. Tick is 0, last_scanned is 0. Next track tick is 6.
    // Since current tick < 6, the target does not need tracking yet.
    // It should perform jamming instead!
    rc.update();

    assert_eq!(rc.radar_states[0], RadarState::Jamming { contact_id: 42 });
    select_radar(0);
    assert_eq!(radar_ecm_mode(), EcmMode::Noise);

    // 2. Set last_scanned and last_measurement_tick to 6 ticks ago.
    // Now next track tick is <= current_tick, so it should trigger a tracking scan.
    rc.contacts[0].last_scanned = current_tick().wrapping_sub(6);
    rc.contacts[0].last_measurement_tick = current_tick().wrapping_sub(6);
    rc.update();

    assert_eq!(rc.radar_states[0], RadarState::Tracking { contact_id: 42 });
    select_radar(0);
    assert_eq!(radar_ecm_mode(), EcmMode::None);
}

#[test]
fn test_missile_priority_scanning_threshold() {
    let mut rc = RadarController::new();

    // Create a prioritized contact moving towards us (velocity -10m/s) so it is not pruned
    let contact = Contact {
        id: 100,
        kinematic: KinematicState::new(Class::Missile, Vec2::new(1000.0, 0.0), Vec2::new(-10.0, 0.0), Vec2::new(0.0, 0.0), current_tick().wrapping_sub(1)),
        last_measurement_tick: current_tick().wrapping_sub(1),
        rssi: -50.0,
        snr: 30.0,
        pos_uncertainty: 10.0,
        vel_uncertainty: 5.0,
        radar_locked: true,
        provisional: false,
        tracking_retry_count: 0,
        confirmation_attempts: 0,
        unscanned_in_range_ticks: 0,
        p_cov_x: Contact::initial_cov(10.0, 5.0, Class::Missile),
        p_cov_y: Contact::initial_cov(10.0, 5.0, Class::Missile),
        prioritize_scan: true,
        prev_scan_pos_uncertainty: Some(10.0),
        low_improvement_consecutive_scans: 0,
        last_beam_width: None,
        last_beam_center: None,
        last_beam_center_pos: None,
        missile_scan_ticks_remaining: 0,
        scan_boundary_points: None,
        scan_boundary_vels: None,
    };
    rc.contacts.push(contact);

    // Call process_scan_hit with a scan that has a very low SNR (e.g. 5.0 dB),
    // causing a very large scan CI and virtually no improvement in the contact's uncertainty.
    let scan_hit_1 = ScanResult {
        position: Vec2::new(990.0, 0.0),
        velocity: Vec2::new(-10.0, 0.0),
        class: Class::Missile,
        rssi: -50.0,
        snr: 5.0,
    };

    rc.process_scan_hit(scan_hit_1, None);

    let updated_contact_1 = rc.get_contact(100).unwrap();
    // It should have low_improvement_consecutive_scans = 1 and still be prioritized
    assert_eq!(updated_contact_1.low_improvement_consecutive_scans, 1);
    assert_eq!(updated_contact_1.prioritize_scan, true);

    // Now update last_scanned and last_measurement_tick so another update can happen
    rc.contacts[0].last_scanned = current_tick().wrapping_sub(1);
    rc.contacts[0].last_measurement_tick = current_tick().wrapping_sub(1);

    // Call process_scan_hit a second time with low SNR scan hit
    let scan_hit_2 = ScanResult {
        position: Vec2::new(980.0, 0.0),
        velocity: Vec2::new(-10.0, 0.0),
        class: Class::Missile,
        rssi: -50.0,
        snr: 5.0,
    };

    rc.process_scan_hit(scan_hit_2, None);

    let updated_contact_2 = rc.get_contact(100).unwrap();
    // It should now have low_improvement_consecutive_scans = 2 and prioritize_scan = false
    assert_eq!(updated_contact_2.low_improvement_consecutive_scans, 2);
    assert_eq!(updated_contact_2.prioritize_scan, false);
}

#[test]
fn test_advanced_beam_tracking() {
    let mut rc = RadarController::new();
    rc.set_tracking_width(0.05);

    // 1. Test upper-bounding of the beam width when last scan was within 0.25s
    let contact_upper_bound = Contact {
        id: 1,
        kinematic: KinematicState::new(Class::Fighter, Vec2::new(1000.0, 0.0), Vec2::new(0.0, 0.0), Vec2::new(0.0, 0.0), current_tick().wrapping_sub(1)),
        last_measurement_tick: current_tick().wrapping_sub(1),
        rssi: -50.0,
        snr: 40.0,
        pos_uncertainty: 1.0,
        vel_uncertainty: 1.0,
        radar_locked: true,
        provisional: false,
        tracking_retry_count: 0,
        confirmation_attempts: 0,
        unscanned_in_range_ticks: 0,
        p_cov_x: Contact::initial_cov(1.0, 1.0, Class::Fighter),
        p_cov_y: Contact::initial_cov(1.0, 1.0, Class::Fighter),
        prioritize_scan: false,
        prev_scan_pos_uncertainty: None,
        low_improvement_consecutive_scans: 0,
        last_beam_width: Some(0.02),
        last_beam_center: Some(0.0),
        last_beam_center_pos: Some(Vec2::new(1000.0, 0.0)),
        missile_scan_ticks_remaining: 0,
        scan_boundary_points: None,
        scan_boundary_vels: None,
    };
    rc.contacts.push(contact_upper_bound.clone());

    let job_upper_bound = rc.generate_tracking_scan(&contact_upper_bound, current_tick()).unwrap();
    assert!(job_upper_bound.width <= 0.02, "Width was {}, expected <= 0.02", job_upper_bound.width);

    // 2. Test high-variance centering logic
    let contact_high_variance = Contact {
        id: 2,
        kinematic: KinematicState::new(Class::Fighter, Vec2::new(1000.0, 50.0), Vec2::new(0.0, 0.0), Vec2::new(0.0, 0.0), current_tick().wrapping_sub(1)),
        last_measurement_tick: current_tick().wrapping_sub(1),
        rssi: -50.0,
        snr: 40.0,
        pos_uncertainty: 10.0,
        vel_uncertainty: 1.0,
        radar_locked: true,
        provisional: false,
        tracking_retry_count: 0,
        confirmation_attempts: 0,
        unscanned_in_range_ticks: 0,
        p_cov_x: Contact::initial_cov(10.0, 1.0, Class::Fighter),
        p_cov_y: Contact::initial_cov(10.0, 1.0, Class::Fighter),
        prioritize_scan: false,
        prev_scan_pos_uncertainty: None,
        low_improvement_consecutive_scans: 0,
        last_beam_width: Some(0.02),
        last_beam_center: Some(0.0),
        last_beam_center_pos: Some(Vec2::new(1000.0, 0.0)),
        missile_scan_ticks_remaining: 0,
        scan_boundary_points: None,
        scan_boundary_vels: None,
    };
    
    let job_high_variance = rc.generate_tracking_scan(&contact_high_variance, current_tick()).unwrap();
    assert!(job_high_variance.angle.abs() < 1e-5, "Angle was {}, expected 0.0", job_high_variance.angle);
}

#[test]
fn test_new_missile_prioritization() {
    let mut rc = RadarController::new();
    rc.new_missile_scan_ticks = 4;

    // 1. Encounter a new incoming missile
    let scan_hit = ScanResult {
        position: Vec2::new(100.0, 0.0),
        velocity: Vec2::new(-10.0, 0.0),
        class: Class::Missile,
        rssi: -50.0,
        snr: 25.0,
    };

    let id = rc.process_scan_hit(scan_hit.clone(), None);
    assert!(id > 0);

    let contact = rc.get_contact(id).unwrap();
    assert_eq!(contact.class, Class::Missile);
    assert_eq!(contact.missile_scan_ticks_remaining, 4);

    // 2. Check that the tracking job has interval = 1 and gets prioritized in tracking_jobs()
    rc.contacts[0].last_scanned = current_tick().wrapping_sub(1);
    rc.contacts[0].last_measurement_tick = current_tick().wrapping_sub(1);
    let jobs: Vec<RadarJob> = rc.tracking_jobs().collect();
    assert_eq!(jobs.len(), 1);
    assert_eq!(jobs[0].state, RadarState::Tracking { contact_id: id });

    // 3. Process matched scans and check that ticks decrement
    // A scan hit on the next tick
    rc.contacts[0].last_scanned = current_tick().wrapping_sub(1);
    rc.contacts[0].last_measurement_tick = current_tick().wrapping_sub(1);
    let scan_hit2 = ScanResult {
        position: Vec2::new(90.0, 0.0),
        velocity: Vec2::new(-10.0, 0.0),
        class: Class::Missile,
        rssi: -50.0,
        snr: 25.0,
    };
    rc.process_scan_hit(scan_hit2, None);
    
    let contact = rc.get_contact(id).unwrap();
    assert_eq!(contact.missile_scan_ticks_remaining, 3);
}

#[test]
fn test_tracking_beam_exclusion() {
    let mut rc = RadarController::new();

    // Target contact 42 (active target)
    let contact_target = Contact {
        id: 42,
        kinematic: KinematicState::new(Class::Fighter, Vec2::new(1000.0, 0.0), Vec2::new(0.0, 0.0), Vec2::new(0.0, 0.0), 0),
        last_measurement_tick: 0,
        rssi: -50.0,
        snr: 40.0,
        pos_uncertainty: 1.0,
        vel_uncertainty: 1.0,
        radar_locked: true,
        provisional: false,
        tracking_retry_count: 0,
        confirmation_attempts: 0,
        unscanned_in_range_ticks: 0,
        p_cov_x: Contact::initial_cov(1.0, 1.0, Class::Fighter),
        p_cov_y: Contact::initial_cov(1.0, 1.0, Class::Fighter),
        prioritize_scan: false,
        prev_scan_pos_uncertainty: None,
        low_improvement_consecutive_scans: 0,
        last_beam_width: None,
        last_beam_center: None,
        last_beam_center_pos: None,
        missile_scan_ticks_remaining: 0,
        scan_boundary_points: None,
        scan_boundary_vels: None,
    };
    rc.contacts.push(contact_target);

    // Contact 43 (outside the beam, but close to where the hit will be simulated)
    let contact_other = Contact {
        id: 43,
        kinematic: KinematicState::new(Class::Fighter, Vec2::new(1000.0, 200.0), Vec2::new(0.0, 0.0), Vec2::new(0.0, 0.0), 0),
        last_measurement_tick: 0,
        rssi: -50.0,
        snr: 40.0,
        pos_uncertainty: 10.0,
        vel_uncertainty: 1.0,
        radar_locked: true,
        provisional: false,
        tracking_retry_count: 0,
        confirmation_attempts: 0,
        unscanned_in_range_ticks: 0,
        p_cov_x: Contact::initial_cov(10.0, 1.0, Class::Fighter),
        p_cov_y: Contact::initial_cov(10.0, 1.0, Class::Fighter),
        prioritize_scan: false,
        prev_scan_pos_uncertainty: None,
        low_improvement_consecutive_scans: 0,
        last_beam_width: None,
        last_beam_center: None,
        last_beam_center_pos: None,
        missile_scan_ticks_remaining: 0,
        scan_boundary_points: None,
        scan_boundary_vels: None,
    };
    rc.contacts.push(contact_other);

    // Tracking beam centered at 0.0, width 0.05
    let slice = Some(ScanSlice {
        angle: 0.0,
        width: 0.05,
        min_distance: 900.0,
        max_distance: 1100.0,
    });

    // Hit position at (1000.0, 180.0) which is 20m from contact 43 (within association gate)
    let scan_hit = ScanResult {
        position: Vec2::new(1000.0, 180.0),
        velocity: Vec2::new(0.0, 0.0),
        class: Class::Fighter,
        rssi: -50.0,
        snr: 30.0,
    };

    // Calculate scan CI radius: SNR = 30.0 gives small error factor, so scan CI radius is around 38m
    let scan_ci_radius = 38.0;

    // 1. Without slice (beam filter) info, contact 43 should match
    let res_no_slice = rc.find_best_matching_contact(&scan_hit, scan_ci_radius, None, 0, 1.0);
    assert_eq!(res_no_slice.map(|(id, _)| id), Some(43));

    // 2. With slice info, contact 43 does not intersect the beam and must be excluded,
    // so no match should be found.
    let res_with_slice = rc.find_best_matching_contact(&scan_hit, scan_ci_radius, slice, 0, 1.0);
    assert_eq!(res_with_slice, None);
}

#[test]
fn test_scan_boundary() {
    let mut contact = Contact {
        id: 1,
        kinematic: KinematicState::new(Class::Fighter, Vec2::new(100.0, 0.0), Vec2::new(10.0, 0.0), Vec2::new(0.0, 0.0), 0),
        last_measurement_tick: 0,
        rssi: 0.0,
        snr: 30.0,
        pos_uncertainty: 10.0,
        vel_uncertainty: 2.0,
        radar_locked: true,
        provisional: false,
        tracking_retry_count: 0,
        confirmation_attempts: 0,
        unscanned_in_range_ticks: 0,
        p_cov_x: Contact::initial_cov(10.0, 2.0, Class::Fighter),
        p_cov_y: Contact::initial_cov(10.0, 2.0, Class::Fighter),
        prioritize_scan: false,
        prev_scan_pos_uncertainty: None,
        low_improvement_consecutive_scans: 0,
        last_beam_width: None,
        last_beam_center: None,
        last_beam_center_pos: None,
        missile_scan_ticks_remaining: 0,
        scan_boundary_points: None,
        scan_boundary_vels: None,
    };

    let slice = ScanSlice {
        angle: 0.0,
        width: 0.1,
        min_distance: 80.0,
        max_distance: 120.0,
    };

    contact.update_scan_boundary(slice);

    assert!(contact.scan_boundary_points.is_some());
    assert!(contact.scan_boundary_vels.is_some());

    let pts_before = contact.scan_boundary_points.unwrap();
    let vels_before = contact.scan_boundary_vels.unwrap();

    // The geometric center of the corners
    let center = (pts_before[0] + pts_before[1] + pts_before[2] + pts_before[3]) / 4.0;
    
    // Check that velocity vector has components directed away from center
    for i in 0..4 {
        let diff = pts_before[i] - center;
        let dir = diff.normalize();
        let expected_v = contact.current_velocity() + dir * (contact.ci_mult() * contact.vel_uncertainty * 2.0f64.sqrt());
        assert!((vels_before[i] - expected_v).length() < 1e-6);
    }

    // Predict
    let dt = 1.0;
    contact.predict_scan_boundary(dt);

    let pts_after = contact.scan_boundary_points.unwrap();
    let vels_after = contact.scan_boundary_vels.unwrap();

    let stats = contact.class.default_stats();
    let mut max_acc = stats
        .max_forward_acceleration
        .max(stats.max_backward_acceleration)
        .max(stats.max_lateral_acceleration);
    if contact.class == Class::Fighter || contact.class == Class::Missile {
        max_acc += 100.0;
    }

    for i in 0..4 {
        let diff = pts_before[i] - center;
        let dir = diff.normalize();
        let expected_v_after = vels_before[i] + dir * (max_acc * 2.0f64.sqrt() * dt);
        assert!((vels_after[i] - expected_v_after).length() < 1e-6);

        let expected_p_after = pts_before[i] + expected_v_after * dt;
        assert!((pts_after[i] - expected_p_after).length() < 1e-6);
    }

    // Verify generate_tracking_scan constraints with the scan boundary
    let mut rc = RadarController::new();
    let mut contact2 = Contact {
        id: 10,
        kinematic: KinematicState::new(Class::Fighter, Vec2::new(1000.0, 0.0), Vec2::new(0.0, 0.0), Vec2::new(0.0, 0.0), 0),
        last_measurement_tick: 0,
        rssi: 0.0,
        snr: 30.0,
        pos_uncertainty: 20.0,
        vel_uncertainty: 5.0,
        radar_locked: true,
        provisional: false,
        tracking_retry_count: 0,
        confirmation_attempts: 0,
        unscanned_in_range_ticks: 0,
        p_cov_x: Contact::initial_cov(20.0, 5.0, Class::Fighter),
        p_cov_y: Contact::initial_cov(20.0, 5.0, Class::Fighter),
        prioritize_scan: false,
        prev_scan_pos_uncertainty: None,
        low_improvement_consecutive_scans: 0,
        last_beam_width: None,
        last_beam_center: None,
        last_beam_center_pos: None,
        missile_scan_ticks_remaining: 0,
        scan_boundary_points: None,
        scan_boundary_vels: None,
    };

    // Initialize scan boundary
    let slice2 = ScanSlice {
        angle: 0.0,
        width: 0.05,
        min_distance: 950.0,
        max_distance: 1050.0,
    };
    contact2.update_scan_boundary(slice2);

    rc.contacts.push(contact2.clone());
    let job = rc.generate_tracking_scan(&contact2, 0).expect("Job should be generated");

    // The job min and max distances must be constrained by the boundary points
    let next_our_pos = position() + velocity() * TICK_LENGTH;
    let pts = contact2.scan_boundary_points.unwrap();
    let vels = contact2.scan_boundary_vels.unwrap();
    let mut min_pt_dist = f64::MAX;
    let mut max_pt_dist = -f64::MAX;
    let mut min_pt_rel_angle = f64::MAX;
    let mut max_pt_rel_angle = -f64::MAX;
    let target_angle = (contact2.position_at(1) - next_our_pos).angle();

    for i in 0..4 {
        let p_proj = pts[i] + vels[i] * TICK_LENGTH;
        let d = next_our_pos.distance(p_proj);
        min_pt_dist = min_pt_dist.min(d);
        max_pt_dist = max_pt_dist.max(d);

        let pt_angle = (p_proj - next_our_pos).angle();
        let rel_angle = normalize_angle(pt_angle - target_angle);
        min_pt_rel_angle = min_pt_rel_angle.min(rel_angle);
        max_pt_rel_angle = max_pt_rel_angle.max(rel_angle);
    }

    assert!(job.min_distance >= min_pt_dist - 1e-6);
    assert!(job.max_distance <= max_pt_dist + 1e-6);

    // The job angle and width bounds must not extend beyond the leftmost/rightmost points (subject to min beam angle 0.005)
    let job_left_rel = normalize_angle((job.angle + job.width / 2.0) - target_angle);
    let job_right_rel = normalize_angle((job.angle - job.width / 2.0) - target_angle);

    if job.width > 0.005 + 1e-9 {
        assert!(job_left_rel <= max_pt_rel_angle + 1e-6);
        assert!(job_right_rel >= min_pt_rel_angle - 1e-6);
    }
}
