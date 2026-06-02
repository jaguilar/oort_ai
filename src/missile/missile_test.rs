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
