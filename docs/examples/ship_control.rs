//! Compilable examples for the Oort Ship Control APIs.
//! Covers position, movement, rotation, limits, and environmental queries.

use oort_api::prelude::*;
use oort_api::ClassStats;


/// Example of querying basic ship telemetry and metadata.
pub fn show_telemetry() {
    // Current position of the ship in meters (world coordinates)
    let pos: Vec2 = position();
    
    // Current velocity of the ship in m/s (world coordinates)
    let vel: Vec2 = velocity();
    
    // Current facing angle of the ship in radians (0 is positive X-axis)
    let head: f64 = heading();
    
    // Current rate of rotation in radians per second
    let ang_vel: f64 = angular_velocity();

    // Unique integer identifier for this ship within the team
    let my_id: u32 = id();

    // The class of the ship (e.g. Class::Fighter, Class::Cruiser)
    let my_class: Class = class();

    // Current health of the ship (starts at max_health, 0 means destroyed)
    let hp: f64 = health();

    // Remaining fuel capacity expressed as delta-V (m/s)
    let current_fuel: f64 = fuel();

    // Retrieve default stats for this class of ship
    let stats: ClassStats = my_class.default_stats();

    // Debug print values to sandbox logs
    debug!("ID: {}, Class: {:?}", my_id, my_class);
    debug!("Pos: {:?}, Vel: {:?}, Heading: {}", pos, vel, head);
    debug!("AngVel: {}, Health: {}, Fuel: {}", ang_vel, hp, current_fuel);
    debug!("Class Max HP: {}", stats.max_health);
}

/// Example of querying environmental variables.
pub fn show_environment() {
    // Unique seed for initializing pseudo-random number generators
    let seed_val: u128 = seed();

    // Name of the active scenario/challenge
    let name: &str = scenario_name();

    // Dimensions of the sandbox world boundaries in meters
    let world_width: f64 = world_size();

    // Total ticks elapsed since the match started (1 tick = 1/60s)
    let tick_count: u32 = current_tick();

    // Total simulated seconds elapsed since the match started
    let elapsed_sec: f64 = current_time();

    // Standard simulation tick interval (fixed at 1/60s)
    let delta_t: f64 = TICK_LENGTH;

    debug!("Scenario: {}, Size: {}m, Seed: {}", name, world_width, seed_val);
    debug!("Tick: {} ({}s), dt: {}", tick_count, elapsed_sec, delta_t);
}

/// Example demonstrating the movement (translation) control APIs.
pub fn show_movement() {
    // Query maximum accelerations available on the ship
    let forward_limit: f64 = max_forward_acceleration();
    let backward_limit: f64 = max_backward_acceleration();
    let lateral_limit: f64 = max_lateral_acceleration();

    // 1. Move forward at maximum speed
    let heading_dir = Vec2::new(heading().cos(), heading().sin());
    accelerate(heading_dir * forward_limit);

    // 2. Stop translation completely by accelerating in the opposite direction of velocity
    if velocity().length() > 0.1 {
        // Accelerate counter to our current motion to brake
        accelerate(-velocity().normalize() * forward_limit);
    } else {
        accelerate(vec2(0.0, 0.0));
    }

    // 3. Lateral movement (strafing right relative to the ship's nose)
    // Rotate the heading direction by -90 degrees (clockwise) to get the right-hand vector
    let right_dir = heading_dir.rotate(-TAU / 4.0);
    accelerate(right_dir * lateral_limit);

    debug!("Limits: Fwd: {}, Bwd: {}, Lat: {}", forward_limit, backward_limit, lateral_limit);
}

/// Example demonstrating rotation and angular control.
pub fn show_rotation() {
    let max_ang_accel: f64 = max_angular_acceleration();

    // 1. Using higher-level turn() API to align to a target heading
    let target_angle = 1.5; // radians (~90 degrees)
    let difference = angle_diff(heading(), target_angle);
    
    // Command the ship to rotate towards the target angle at a target rate
    // proportional to the angular distance (clamped to limits)
    turn(difference * 5.0);

    // 2. Using low-level torque() API to directly control angular acceleration (radians/s²)
    // This allows custom PD (Proportional-Derivative) controllers for faster, precise turning
    let kp = 10.0;
    let kd = 2.0;
    let commanded_torque = kp * difference - kd * angular_velocity();
    
    // Set direct torque, clamped to the physical limitations of the ship
    torque(commanded_torque.clamp(-max_ang_accel, max_ang_accel));

    debug!("Max Angular Accel: {}", max_ang_accel);
}

/// 3. Optimal High-Speed Turning (Maximum Acceleration Deceleration-Limiting Controller)
pub fn quick_turn(target_angle: f64) {
    let difference = angle_diff(heading(), target_angle);
    let omega = angular_velocity();
    let max_ang_accel = max_angular_acceleration();
    
    // Safety buffer: use 98% of max angular acceleration to prevent any overshoot
    let a_dec = max_ang_accel * 0.98;
    let k_p = 10.0;
    
    let theta_trans = a_dec / (k_p * k_p);
    let theta_offset = theta_trans / 2.0;
    
    let omega_target = if difference.abs() <= theta_trans {
        k_p * difference
    } else {
        difference.signum() * (2.0 * a_dec * (difference.abs() - theta_offset)).sqrt()
    };
    
    let alpha_req = (omega_target - omega) / TICK_LENGTH;
    torque(alpha_req.clamp(-max_ang_accel, max_ang_accel));
}
