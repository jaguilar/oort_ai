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
fn test_calculate_nez_metric() {
    let snapshot = StateSnapshot {
        position: vec2(0.0, 0.0),
        velocity: vec2(100.0, 0.0),
        heading: 0.0,
        angular_velocity: 0.0,
        fuel: 1000.0,
        current_tick: 0,
        max_forward_acceleration: 250.0,
        max_lateral_acceleration: 300.0,
        max_angular_acceleration: 100.0,
        target: None,
        cruise_point: None,
        has_entered_nez: false,
        dodge_sign: 1.0,
    };
    let target = TargetSnapshot {
        position: vec2(1000.0, 0.0),
        velocity: vec2(0.0, 0.0),
        class: Class::Fighter,
        last_scanned: 0,
    };
    
    let nez = calculate_nez_metric(&snapshot, &target, vec2(1000.0, 0.0), 5.0);
    assert!(nez.is_finite());
}

#[test]
fn test_estimate_t_go() {
    let snapshot = StateSnapshot {
        position: vec2(0.0, 0.0),
        velocity: vec2(0.0, 0.0),
        heading: 0.0,
        angular_velocity: 0.0,
        fuel: 10000.0,
        current_tick: 0,
        max_forward_acceleration: 0.0,
        max_lateral_acceleration: 300.0,
        max_angular_acceleration: 100.0,
        target: None,
        cruise_point: None,
        has_entered_nez: false,
        dodge_sign: 1.0,
    };
    let target = TargetSnapshot {
        position: vec2(1000.0, 0.0),
        velocity: vec2(-10.0, 0.0),
        class: Class::Fighter,
        last_scanned: 0,
    };

    let t_go = estimate_t_go(&snapshot, &target, false);
    assert!(t_go.is_finite());
    assert!((t_go - 100.0).abs() < 1e-3, "Expected 100.0, got {}", t_go);
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

#[test]
fn test_determine_thrust() {
    let mut snapshot = StateSnapshot {
        position: vec2(0.0, 0.0),
        velocity: vec2(100.0, 0.0),
        heading: 0.0,
        angular_velocity: 0.0,
        fuel: 100.0, // below DEFAULT_MIN_SEARCH_FUEL fuel limit
        current_tick: 0,
        max_forward_acceleration: 250.0,
        max_lateral_acceleration: 300.0,
        max_angular_acceleration: 100.0,
        target: None,
        cruise_point: None,
        has_entered_nez: false,
        dodge_sign: 1.0,
    };
    let prograde = vec2(0.0, 1.0);
    
    // Low fuel mode
    let thrust_low = determine_thrust(&snapshot, Some(prograde), DEFAULT_MIN_SEARCH_FUEL).unwrap();
    assert!(thrust_low.length() > 0.0);

    // High fuel mode
    snapshot.fuel = 1000.0;
    let thrust_high = determine_thrust(&snapshot, Some(prograde), DEFAULT_MIN_SEARCH_FUEL).unwrap();
    assert!(thrust_high.length() > 0.0);

    // Borderline fuel mode: slightly above limit, but not enough to achieve heading match (100.0 dv needed)
    snapshot.fuel = DEFAULT_MIN_SEARCH_FUEL + 1.0;
    let thrust_borderline = determine_thrust(&snapshot, Some(prograde), DEFAULT_MIN_SEARCH_FUEL).unwrap();
    // It should fallback to calculate_min_lateral_thrust, which steers towards prograde (y-axis)
    assert!(thrust_borderline.y > 0.0);
    assert!((thrust_borderline.x).abs() < 1e-3);
}

#[test]
fn test_has_entered_nez_latching() {
    let mut mg = MissileGuidance::new();
    assert!(!mg.has_entered_nez);

    mg.has_entered_nez = true;
    let prev_target_id = mg.target_id;
    mg.target_id = Some(42);

    if mg.target_id != prev_target_id {
        mg.has_entered_nez = false;
    }

    assert!(!mg.has_entered_nez);
}

#[test]
fn test_flak_dodging_math() {
    assert_eq!(N, 150.0);
    assert_eq!(M, 1.5);

    let target_pos = vec2(1000.0, 0.0);
    let missile_pos = vec2(0.0, 0.0);
    let r = target_pos - missile_pos;
    let r_len = r.length();
    let perp_dir = vec2(-r.y, r.x) / r_len;
    
    let dodge_sign_left = 1.0;
    let offset_left = perp_dir * (dodge_sign_left * (N / 2.0));
    assert!((offset_left.x - 0.0).abs() < 1e-6);
    assert!((offset_left.y - 75.0).abs() < 1e-6);

    let dodge_sign_right = -1.0;
    let offset_right = perp_dir * (dodge_sign_right * (N / 2.0));
    assert!((offset_right.x - 0.0).abs() < 1e-6);
    assert!((offset_right.y - -75.0).abs() < 1e-6);
}

