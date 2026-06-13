use super::*;

struct Lcg {
    state: u64,
}

impl Lcg {
    fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        self.state
    }

    fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    fn gen_range(&mut self, min: f64, max: f64) -> f64 {
        min + self.next_f64() * (max - min)
    }

    fn gen_vec2(&mut self, min_mag: f64, max_mag: f64) -> (f64, f64) {
        let mag = self.gen_range(min_mag, max_mag);
        let angle = self.gen_range(-std::f64::consts::PI, std::f64::consts::PI);
        (mag * angle.cos(), mag * angle.sin())
    }
}

fn wrap_angle(angle: f64) -> f64 {
    let mut a = angle % (2.0 * std::f64::consts::PI);
    if a > std::f64::consts::PI {
        a -= 2.0 * std::f64::consts::PI;
    } else if a < -std::f64::consts::PI {
        a += 2.0 * std::f64::consts::PI;
    }
    a
}

fn angle_diff_local(from: f64, to: f64) -> f64 {
    let diff = (to - from) % (2.0 * std::f64::consts::PI);
    let diff = if diff > std::f64::consts::PI {
        diff - 2.0 * std::f64::consts::PI
    } else if diff < -std::f64::consts::PI {
        diff + 2.0 * std::f64::consts::PI
    } else {
        diff
    };
    diff
}

fn run_simulation_test_cases<F>(torque_controller: F, name: &str)
where
    F: Fn(f64, f64, f64, Vec2, Vec2, Vec2, Vec2, Vec2, Vec2, f64) -> f64,
{
    let mut lcg = Lcg::new(1337);
    let max_torque = 2.0 * std::f64::consts::PI;
    let max_accel = 60.0;
    let dt = 1.0 / 60.0;
    let epsilon = 1e-4;
    let max_ticks = 240;

    let mut successes = 0;
    let mut failures = 0;
    let mut max_torque_failures = 0;
    let mut overshoot_failures = 0;
    let mut convergence_failures = 0;

    #[derive(Debug)]
    struct FailureInfo {
        case_idx: usize,
        reason: &'static str,
        turn_ticks: usize,
        max_torque_ticks_before_align: usize,
        min_required: usize,
        has_overshot: bool,
        unavoidable: bool,
        max_overshoot: f64,
        init_err: f64,
        init_omega_rel: f64,
    }
    let mut failure_details = Vec::new();

    for case_idx in 0..1000 {
        let mut p_pos = Vec2::new(0.0, 0.0);

        let dist = lcg.gen_range(1000.0, 20000.0);
        let target_start_angle = lcg.gen_range(-std::f64::consts::PI, std::f64::consts::PI);
        let mut t_pos = Vec2::new(dist * target_start_angle.cos(), dist * target_start_angle.sin());

        let (p_vx, p_vy) = lcg.gen_vec2(0.0, 100.0);
        let mut p_vel = Vec2::new(p_vx, p_vy);

        let (t_vx, t_vy) = lcg.gen_vec2(0.0, 100.0);
        let mut t_vel = Vec2::new(t_vx, t_vy);

        let (t_ax, t_ay) = lcg.gen_vec2(0.0, max_accel);
        let t_accel = Vec2::new(t_ax, t_ay);

        let mut p_omega = lcg.gen_range(-max_torque, max_torque);
        let mut p_heading = lcg.gen_range(-std::f64::consts::PI, std::f64::consts::PI);

        let p_accel_mode = case_idx % 3;
        let (p_ax_const, p_ay_const) = if p_accel_mode == 1 {
            lcg.gen_vec2(0.0, max_accel)
        } else {
            (0.0, 0.0)
        };
        let p_accel_const = Vec2::new(p_ax_const, p_ay_const);

        let mut converged = false;
        let mut consecutive_aligned = 0;
        let mut turn_ticks = 0;

        // Track initial conditions for overshoot checking
        let initial_r = t_pos - p_pos;
        let initial_v_rel = t_vel - p_vel;
        let initial_target_heading = initial_r.angle();
        let initial_target_omega = if initial_r.dot(initial_r) > 1e-6 {
            (initial_r.x * initial_v_rel.y - initial_r.y * initial_v_rel.x) / initial_r.dot(initial_r)
        } else {
            0.0
        };
        let initial_error = angle_diff_local(p_heading, initial_target_heading);
        let initial_omega_rel = p_omega - initial_target_omega;

        let mut torques = Vec::new();
        let mut has_overshot = false;
        let mut max_overshoot: f64 = 0.0;
        let mut overshoot_tick = None;
        let mut prev_error: Option<f64> = None;

        for tick in 0..max_ticks {
            let r = t_pos - p_pos;
            let target_heading = r.angle();

            let heading_error = angle_diff_local(p_heading, target_heading);

            if heading_error.abs() <= epsilon {
                consecutive_aligned += 1;
            } else {
                consecutive_aligned = 0;
            }

            if consecutive_aligned >= 3 {
                if !converged {
                    converged = true;
                    turn_ticks = tick - 2; // First tick of alignment
                }
            }

            // Overshoot detection: sign change of heading error
            if let Some(prev_err) = prev_error {
                if prev_err * heading_error < 0.0 && heading_error.abs() < 0.2 {
                    has_overshot = true;
                    if overshoot_tick.is_none() {
                        overshoot_tick = Some(tick);
                    }
                }
            }
            prev_error = Some(heading_error);

            if has_overshot {
                max_overshoot = max_overshoot.max(heading_error.abs());
            }

            // Player acceleration
            let p_accel = match p_accel_mode {
                0 => Vec2::new(0.0, 0.0),
                1 => p_accel_const,
                _ => Vec2::new(max_accel * p_heading.cos(), max_accel * p_heading.sin()),
            };

            let torque_cmd = torque_controller(
                p_heading,
                p_omega,
                max_torque,
                p_pos,
                p_vel,
                p_accel,
                t_pos,
                t_vel,
                t_accel,
                dt,
            );

            torques.push(torque_cmd);

            // Update target physics
            t_vel = t_vel + t_accel * dt;
            t_pos = t_pos + t_vel * dt;

            // Update player physics
            p_vel = p_vel + p_accel * dt;
            p_pos = p_pos + p_vel * dt;

            let torque = torque_cmd.clamp(-max_torque, max_torque);
            p_omega = p_omega + torque * dt;
            p_heading = wrap_angle(p_heading + p_omega * dt);
        }

        if converged {
            let mut is_valid = true;
            let mut reason = "";

            // Check if overshoot was unavoidable (max deceleration commanded on all ticks up to the overshoot)
            let unavoidable_overshoot = if has_overshot {
                if let Some(ot) = overshoot_tick {
                    torques.iter().take(ot + 1).all(|&tq| {
                        if initial_omega_rel > 0.0 {
                            tq <= -max_torque + 1e-9
                        } else if initial_omega_rel < 0.0 {
                            tq >= max_torque - 1e-9
                        } else {
                            false
                        }
                    })
                } else {
                    false
                }
            } else {
                false
            };

            if has_overshot && !unavoidable_overshoot && max_overshoot > 0.35 {
                overshoot_failures += 1;
                is_valid = false;
                reason = "Overshoot";
            }

            // Calculate minimum required max torque ticks during the active turning phase
            let min_required_max_torque_ticks = (0.9 * turn_ticks as f64).max((turn_ticks as isize - 6) as f64).ceil() as usize;

            let mut max_torque_ticks_before_align = 0;
            for &tq in torques.iter().take(turn_ticks) {
                if tq.abs() >= max_torque - 1e-9 {
                    max_torque_ticks_before_align += 1;
                }
            }

            if max_torque_ticks_before_align < min_required_max_torque_ticks {
                max_torque_failures += 1;
                is_valid = false;
                reason = if reason.is_empty() { "Max Torque Ticks" } else { "Overshoot & Max Torque" };
            }

            if is_valid {
                successes += 1;
            } else {
                failures += 1;
                failure_details.push(FailureInfo {
                    case_idx,
                    reason,
                    turn_ticks,
                    max_torque_ticks_before_align,
                    min_required: min_required_max_torque_ticks,
                    has_overshot,
                    unavoidable: unavoidable_overshoot,
                    max_overshoot,
                    init_err: initial_error,
                    init_omega_rel: initial_omega_rel,
                });
            }
        } else {
            failures += 1;
            convergence_failures += 1;
            failure_details.push(FailureInfo {
                case_idx,
                reason: "Did Not Converge",
                turn_ticks: max_ticks,
                max_torque_ticks_before_align: 0,
                min_required: 0,
                has_overshot,
                unavoidable: false,
                max_overshoot,
                init_err: initial_error,
                init_omega_rel: initial_omega_rel,
            });
        }
    }

    println!("Successful {} test cases: {} / 1000", name, successes);
    println!("Failures: {}, Max Torque Failures: {}, Overshoot Failures: {}, Convergence Failures: {}",
        failures, max_torque_failures, overshoot_failures, convergence_failures);
    
    for (i, detail) in failure_details.iter().take(50).enumerate() {
        println!("Failure #{}: {:?}", i, detail);
    }

    if failures > 0 {
        panic!("{} test failed: {} test cases failed the requirements.", name, failures);
    }
}

fn run_perpendicular_traversing_test<F>(torque_controller: F, name: &str)
where
    F: Fn(f64, f64, f64, Vec2, Vec2, Vec2, Vec2, Vec2, Vec2, f64) -> f64,
{
    let mut lcg = Lcg::new(42);
    let max_torque = 2.0 * std::f64::consts::PI;
    let dt = 1.0 / 60.0;
    let epsilon = 1e-4;
    let max_ticks = 600; // 10 seconds simulation

    for case_idx in 0..20 {
        // Ship starts at origin, stationary, with randomized initial heading
        let p_pos = Vec2::new(0.0, 0.0);
        let init_heading = lcg.gen_range(-std::f64::consts::PI, std::f64::consts::PI);
        let mut p_heading = init_heading;
        let mut p_omega = 0.0;

        // Enemy starts 15 km away along X axis (heading 0.0)
        // Traversing perpendicular (along Y axis) at randomized speed between 100 and 300 m/s
        let speed = lcg.gen_range(100.0, 300.0);
        let direction = if lcg.next_f64() > 0.5 { 1.0 } else { -1.0 };
        let mut t_pos = Vec2::new(15000.0, 0.0);
        let t_vel = Vec2::new(0.0, direction * speed);

        let mut converged = false;
        let mut consecutive_aligned = 0;
        let mut turn_ticks = None;

        for tick in 0..max_ticks {
            let r = t_pos - p_pos;
            let target_heading = r.angle();
            let r_len_sq = r.dot(r);
            let target_omega = if r_len_sq > 1e-6 {
                (r.x * t_vel.y - r.y * t_vel.x) / r_len_sq
            } else {
                0.0
            };

            let heading_error = angle_diff_local(p_heading, target_heading);
            let omega_error = p_omega - target_omega;

            // We consider the tracking successful if the error is extremely small
            if heading_error.abs() <= epsilon && omega_error.abs() <= 1e-4 {
                consecutive_aligned += 1;
            } else {
                consecutive_aligned = 0;
            }

            if consecutive_aligned >= 10 && turn_ticks.is_none() {
                converged = true;
                turn_ticks = Some(tick - 9);
            }

            let torque_cmd = torque_controller(
                p_heading,
                p_omega,
                max_torque,
                p_pos,
                Vec2::new(0.0, 0.0),
                Vec2::new(0.0, 0.0),
                t_pos,
                t_vel,
                Vec2::new(0.0, 0.0),
                dt,
            );

            // Update physics
            // Enemy moves at a fixed velocity
            t_pos = t_pos + t_vel * dt;

            // Ship only rotates (p_pos is stationary)
            let torque = torque_cmd.clamp(-max_torque, max_torque);
            p_omega = p_omega + torque * dt;
            p_heading = wrap_angle(p_heading + p_omega * dt);
        }

        assert!(
            converged,
            "Case {}: Should converge to target angle and track it with zero lag. Init heading: {:.4} rad, speed: {:.1} m/s",
            case_idx, init_heading, speed
        );
        let tt = turn_ticks.unwrap();
        println!(
            "Case {}: {} converged in {} ticks ({:.2}s). Init heading: {:.4} rad, speed: {:.1} m/s",
            case_idx, name, tt, tt as f64 * dt, init_heading, speed
        );
    }
}

#[test]
fn test_quick_turn_torque_simulation() {
    run_simulation_test_cases(
        |p_heading, p_omega, max_torque, p_pos, p_vel, _p_accel, t_pos, t_vel, _t_accel, dt| {
            let r = t_pos - p_pos;
            let v_rel = t_vel - p_vel;
            let target_heading = r.angle();
            let r_len_sq = r.dot(r);
            let target_omega = if r_len_sq > 1e-6 {
                (r.x * v_rel.y - r.y * v_rel.x) / r_len_sq
            } else {
                0.0
            };
            let target_heading_next = target_heading + target_omega * dt;
            quick_turn_torque_with_target_omega_impl(
                p_heading,
                p_omega,
                max_torque,
                target_heading_next,
                target_omega,
            )
        },
        "Target Omega Simulation",
    );
}

#[test]
fn test_quick_turn_torque_kinematic_simulation() {
    run_simulation_test_cases(
        |p_heading, p_omega, max_torque, p_pos, p_vel, p_accel, t_pos, t_vel, t_accel, _dt| {
            quick_turn_torque_kinematic_impl(
                p_heading,
                p_omega,
                max_torque,
                p_pos,
                p_vel,
                p_accel,
                t_pos,
                t_vel,
                t_accel,
            )
        },
        "Kinematic Simulation",
    );
}

#[test]
fn test_quick_turn_torque_perpendicular_traversing() {
    run_perpendicular_traversing_test(
        |p_heading, p_omega, max_torque, p_pos, p_vel, _p_accel, t_pos, t_vel, _t_accel, dt| {
            let r = t_pos - p_pos;
            let v_rel = t_vel - p_vel;
            let target_heading = r.angle();
            let r_len_sq = r.dot(r);
            let target_omega = if r_len_sq > 1e-6 {
                (r.x * v_rel.y - r.y * v_rel.x) / r_len_sq
            } else {
                0.0
            };
            let target_heading_next = target_heading + target_omega * dt;
            quick_turn_torque_with_target_omega_impl(
                p_heading,
                p_omega,
                max_torque,
                target_heading_next,
                target_omega,
            )
        },
        "Target Omega Perpendicular Traversing",
    );
}

#[test]
fn test_quick_turn_torque_kinematic_perpendicular_traversing() {
    run_perpendicular_traversing_test(
        |p_heading, p_omega, max_torque, p_pos, p_vel, p_accel, t_pos, t_vel, t_accel, _dt| {
            quick_turn_torque_kinematic_impl(
                p_heading,
                p_omega,
                max_torque,
                p_pos,
                p_vel,
                p_accel,
                t_pos,
                t_vel,
                t_accel,
            )
        },
        "Kinematic Perpendicular Traversing",
    );
}

#[test]
fn test_max_achievable_velocity() {
    use crate::physics::KinematicState;
    let state = KinematicState::new(
        Class::Missile,
        vec2(0.0, 0.0),
        vec2(100.0, 50.0),
        vec2(0.0, 0.0),
        0,
    );
    let heading = vec2(1.0, 0.0); // directly x-axis

    // Case 1: available_dv is smaller than perpendicular velocity (50.0)
    let v_res1 = super::max_achievable_velocity(&state, heading, 30.0);
    assert!(v_res1.is_none());

    // Case 2: available_dv is larger than perpendicular velocity (50.0)
    let v_res2 = super::max_achievable_velocity(&state, heading, 130.0).unwrap();
    // 50.0 is used to zero out perp component, leaving 120.0 to gain speed along heading
    // discriminant = sqrt(130^2 - 50^2) = 120
    // v_desired_mag = 100.0 + 120.0 = 220.0
    assert!((v_res2.y - 0.0).abs() < 1e-3);
    assert!((v_res2.x - 220.0).abs() < 1e-3);
}

#[test]
fn test_match_velocity_thrust_heading() {
    let current_vel = vec2(100.0, 0.0);
    let target_vel = vec2(150.0, 50.0);

    let thrust_dir = super::match_velocity_thrust_heading(current_vel, target_vel);
    assert!(thrust_dir.is_some());
    let dir = thrust_dir.unwrap();
    assert!((dir.length() - 1.0).abs() < 1e-6);
    assert!(dir.x > 0.0 && dir.y > 0.0);

    let no_thrust = super::match_velocity_thrust_heading(current_vel, current_vel);
    assert!(no_thrust.is_none());
}

