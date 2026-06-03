use super::*;

#[test]
fn test_target_telemetry_serialization() {
    let telemetry = TargetTelemetry {
        position: vec2(12345.67, -9876.54),
        velocity: vec2(-456.78, 987.65),
        rssi: -45.67,
        class: Class::Fighter,
        tick: 123,
    };
    let payload = telemetry.serialize();
    let deserialized = TargetTelemetry::deserialize(&payload);
    
    assert!((telemetry.position.x - deserialized.position.x).abs() < 1e-1);
    assert!((telemetry.position.y - deserialized.position.y).abs() < 1e-1);
    assert!((telemetry.velocity.x - deserialized.velocity.x).abs() < 1e-2);
    assert!((telemetry.velocity.y - deserialized.velocity.y).abs() < 1e-2);
    assert!((telemetry.rssi - deserialized.rssi).abs() < 1e-3);
    assert_eq!(telemetry.tick, deserialized.tick);
    assert_eq!(telemetry.class, deserialized.class);
}

#[test]
fn test_missile_guidance_math() {
    // Test acceleration weight function
    let weight = |t_go: f64| {
        if t_go >= 5.0 {
            0.0
        } else if t_go < 3.0 {
            1.0
        } else {
            (5.0 - t_go) / 2.0
        }
    };
    assert_eq!(weight(6.0), 0.0);
    assert_eq!(weight(5.0), 0.0);
    assert_eq!(weight(4.0), 0.5);
    assert_eq!(weight(3.0), 1.0);
    assert_eq!(weight(2.0), 1.0);

    // Test alpha calculation to cancel transverse velocity
    let calculate_alpha = |v_perp: f64, v_boost: f64| {
        if v_perp.abs() <= v_boost {
            (-v_perp / v_boost).asin()
        } else {
            -v_perp.signum() * std::f64::consts::FRAC_PI_2
        }
    };
    // If v_perp is 0, alpha should be 0
    assert_eq!(calculate_alpha(0.0, 100.0), 0.0);
    // If v_perp is 50.0 and v_boost is 100.0, sin(alpha) should be -0.5, so alpha = -pi/6
    assert!((calculate_alpha(50.0, 100.0) - (-std::f64::consts::FRAC_PI_6)).abs() < 1e-6);
    // If v_perp is -50.0, alpha = pi/6
    assert!((calculate_alpha(-50.0, 100.0) - std::f64::consts::FRAC_PI_6).abs() < 1e-6);
    // If v_perp is larger than v_boost, it should clamp to -pi/2
    assert_eq!(calculate_alpha(150.0, 100.0), -std::f64::consts::FRAC_PI_2);
}

#[test]
fn test_calculate_nez_metric() {
    let mg = MissileGuidance::new();
    let contact = Contact {
        id: 1,
        kinematic: KinematicState::new(Class::Fighter, vec2(1000.0, 0.0), vec2(0.0, 0.0), vec2(0.0, 0.0), 0),
        rssi: 0.0,
        snr: 30.0,
        pos_uncertainty: 0.0,
        vel_uncertainty: 0.0,
        radar_locked: true,
        provisional: false,
        tracking_retry_count: 0,
        confirmation_attempts: 0,
        unscanned_in_range_ticks: 0,
        p_cov_x: [[0.0; 3]; 3],
        p_cov_y: [[0.0; 3]; 3],
    };
    
    let nez = mg.calculate_nez_metric(&contact, vec2(1000.0, 0.0), 5.0);
    // Ensure calculation completes successfully
    assert!(nez.is_finite());
}

#[test]
fn test_missile_message_serialization() {
    // 1. Test Telemetry variant
    let telemetry = TargetTelemetry {
        position: vec2(12345.67, -9876.54),
        velocity: vec2(-456.78, 987.65),
        rssi: -45.67,
        class: Class::Fighter,
        tick: 123,
    };
    let msg_tel = MissileMessage::Telemetry(telemetry);
    let payload_tel = msg_tel.serialize();
    let deserialized_tel = MissileMessage::deserialize(&payload_tel);
    
    if let MissileMessage::Telemetry(t) = deserialized_tel {
        assert!((telemetry.position.x - t.position.x).abs() < 1e-1);
        assert!((telemetry.position.y - t.position.y).abs() < 1e-1);
        assert!((telemetry.velocity.x - t.velocity.x).abs() < 1e-2);
        assert!((telemetry.velocity.y - t.velocity.y).abs() < 1e-2);
        assert!((telemetry.rssi - t.rssi).abs() < 1e-3);
        assert_eq!(telemetry.tick, t.tick);
        assert_eq!(telemetry.class, t.class);
    } else {
        panic!("Deserialized as wrong variant");
    }

    // 2. Test Loiter variant
    let loiter = LoiterCommand {
        aim_point: vec2(-5000.0, 7500.0),
        cruise_speed: 650.0,
    };
    let msg_loi = MissileMessage::Loiter(loiter);
    let payload_loi = msg_loi.serialize();
    let deserialized_loi = MissileMessage::deserialize(&payload_loi);

    if let MissileMessage::Loiter(l) = deserialized_loi {
        assert!((loiter.aim_point.x - l.aim_point.x).abs() < 1e-1);
        assert!((loiter.aim_point.y - l.aim_point.y).abs() < 1e-1);
        assert!((loiter.cruise_speed - l.cruise_speed).abs() < 1e-2);
    } else {
        panic!("Deserialized as wrong variant");
    }
}

#[test]
fn test_estimate_t_go() {
    let mut mg = MissileGuidance::new();
    
    // Test non-cruising mode: Target moving towards stationary missile at 10 m/s from 1000m away.
    // Collision should occur in exactly 100 seconds.
    let contact = Contact {
        id: 1,
        kinematic: KinematicState::new(Class::Fighter, vec2(1000.0, 0.0), vec2(-10.0, 0.0), vec2(0.0, 0.0), 0),
        rssi: 0.0,
        snr: 30.0,
        pos_uncertainty: 0.0,
        vel_uncertainty: 0.0,
        radar_locked: true,
        provisional: false,
        tracking_retry_count: 0,
        confirmation_attempts: 0,
        unscanned_in_range_ticks: 0,
        p_cov_x: [[0.0; 3]; 3],
        p_cov_y: [[0.0; 3]; 3],
    };

    mg.is_cruising = false;
    let t_go_non_cruise = mg.estimate_t_go(&contact);
    assert!(t_go_non_cruise.is_finite());
    // Should be close to 100.0 seconds
    assert!((t_go_non_cruise - 100.0).abs() < 1e-3, "Expected 100.0, got {}", t_go_non_cruise);

    // Test cruising mode
    mg.is_cruising = true;
    let t_go_cruise = mg.estimate_t_go(&contact);
    assert!(t_go_cruise.is_finite());
    assert!(t_go_cruise >= 0.0);
}

#[test]
fn test_minimum_effort_intercept() {
    let missile = KinematicState::new(
        Class::Missile,
        Vec2::new(0.0, 0.0),
        Vec2::new(100.0, 0.0),
        Vec2::new(0.0, 0.0),
        0,
    );
    let enemy = KinematicState::new(
        Class::Fighter,
        Vec2::new(1000.0, 0.0),
        Vec2::new(0.0, 10.0),
        Vec2::new(0.0, 0.0),
        0,
    );

    let res = minimum_effort_intercept(&missile, &enemy, 10.0);
    assert!(res.is_some());
    let data = res.unwrap();
    assert!(data.constant_velocity.t_go > 0.0);
    assert!(data.worst_case_positive.t_go > 0.0);
    assert!(data.worst_case_negative.t_go > 0.0);

    let enemy_away = KinematicState::new(
        Class::Fighter,
        Vec2::new(1000.0, 0.0),
        Vec2::new(200.0, 0.0),
        Vec2::new(0.0, 0.0),
        0,
    );
    let res_away = minimum_effort_intercept(&missile, &enemy_away, 10.0);
    assert!(res_away.is_none());
}


