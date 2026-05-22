# Chapter 1: Ship Physics & Control

Autopilot in Oort requires a solid grasp of 2D space flight dynamics, vector translations, and rotational controllers. This chapter describes the core ship status variables, physics limitations, environmental variables, and movement controllers.

For the corresponding checkable source code, see [ship_control.rs](file:///home/jaguilar/projects/oort_ai/docs/examples/ship_control.rs).

---

## 🛰️ Telemetry and Status API

Each tick (1/60th of a second), the Oort simulation environment updates the physical state of your ship. You can query these values using the following functions:

| Function | Return Type | Description |
|---|---|---|
| `position()` | `Vec2` | Current absolute coordinates of the ship in meters (world space). |
| `velocity()` | `Vec2` | Current absolute velocity of the ship in m/s (world space). |
| `heading()` | `f64` | Ship nose direction in radians. `0.0` points right (positive X-axis), growing counter-clockwise up to `TAU`. |
| `angular_velocity()` | `f64` | Rate of rotation in radians/sec. Positive is counter-clockwise, negative is clockwise. |
| `id()` | `u32` | Unique integer ID of the ship, unique within your team. |
| `class()` | `Class` | The structural class of the ship (e.g. `Class::Fighter`, `Class::Cruiser`, `Class::Frigate`). |
| `health()` | `f64` | Current health of the ship. 0 means destroyed. |
| `fuel()` | `f64` | Remaining fuel capacity expressed as delta-V in m/s (how much total velocity modification is left). |

> [!NOTE]
> Physical limitations vary drastically by `Class`. You can inspect default capabilities using `class().default_stats()`, which returns a `ClassStats` struct.

---

## 🏃 Translation and Movement

To move your ship, you command a linear acceleration vector for the next simulation tick.

* **`accelerate(acceleration: Vec2)`**
  * Commands a linear acceleration vector in **world coordinates** (m/s²).
  * The Oort engine automatically clamps the acceleration to the ship's physical capability.

### Acceleration Limits

* **`max_forward_acceleration() -> f64`**: Max acceleration forward (along `heading()`).
* **`max_backward_acceleration() -> f64`**: Max acceleration backward (opposite to `heading()`).
* **`max_lateral_acceleration() -> f64`**: Max acceleration sideways (orthogonal to `heading()`).

> [!IMPORTANT]
> Because limits are relative to the ship nose, `accelerate()` rotates your vector internally into ship-local coordinates, clamps it to the limits, and then applies the acceleration.

---

## 🔄 Rotation and Angular Control

Oort provides both a high-level rotational helper and low-level torque controls:

1. **`turn(angular_speed: f64)`**
   * Commands the ship to rotate towards a target angular speed in radians/sec.
   * *Limitation:* The ship takes time to accelerate up to the requested speed based on inertia and maximum torque.
2. **`torque(angular_acceleration: f64)`**
   * Commands a direct angular acceleration in radians/sec² for the next tick.
   * This is lower-level than `turn()` and is essential for implementing precise Proportional-Derivative (PD) orientation controllers.
   * Clamped by **`max_angular_acceleration() -> f64`**.

> [!TIP]
> **Optimal High-Speed Turning (`quick_turn`):**
> To turn the ship at the maximum possible speed without overshooting, you can track a composite deceleration-bounded target velocity profile $\omega_{\text{target}}(\Delta\theta)$ and convert it to a discrete torque command:
> 
> ```rust
> pub fn quick_turn(target_angle: f64) {
>     let difference = angle_diff(heading(), target_angle);
>     let omega = angular_velocity();
>     let max_ang_accel = max_angular_acceleration();
>     
>     let a_dec = max_ang_accel * 0.98; // 98% deceleration limit (safety buffer)
>     let k_p = 10.0; // Near-target linear region gain
>     let theta_trans = a_dec / (k_p * k_p);
>     let theta_offset = theta_trans / 2.0;
>     
>     let omega_target = if difference.abs() <= theta_trans {
>         k_p * difference
>     } else {
>         difference.signum() * (2.0 * a_dec * (difference.abs() - theta_offset)).sqrt()
>     };
>     
>     let alpha_req = (omega_target - omega) / TICK_LENGTH;
>     torque(alpha_req.clamp(-max_ang_accel, max_ang_accel));
> }
> ```

---

## 🌍 Environment and Timing Queries

You can query general simulation state parameters using these utilities:

* **`seed() -> u128`**: A per-match unique seed, ideal for seeding random number generators.
* **`scenario_name() -> &'static str`**: Name of the active Oort challenge (e.g., `"tutorial_guns"`).
* **`world_size() -> f64`**: Side length of the square sandbox in meters.
* **`current_tick() -> u32`**: Number of simulation frames elapsed since the start of the match.
* **`current_time() -> f64`**: Total simulated seconds elapsed (`current_tick() * TICK_LENGTH`).
* **`TICK_LENGTH`** (constant): Frame duration, fixed at `1.0 / 60.0` seconds.

---

## 💻 Code Examples

Below is a self-contained demonstration of telemetry querying, flight translation, and rotational tracking.

```rust
use oort_api::prelude::*;
use oort_api::ClassStats;

// Querying telemetry and ship limits
pub fn show_telemetry() {
    let pos: Vec2 = position();
    let vel: Vec2 = velocity();
    let head: f64 = heading();
    let ang_vel: f64 = angular_velocity();
    let hp: f64 = health();
    let current_fuel: f64 = fuel();
    
    let my_class: Class = class();
    let stats: ClassStats = my_class.default_stats();

    debug!("Pos: {:?}, Vel: {:?}, HP: {}, Fuel: {}", pos, vel, hp, current_fuel);
    debug!("Class Max HP: {}", stats.max_health);
}

// Translating the ship (Moving)
pub fn show_movement() {
    let forward_limit = max_forward_acceleration();
    let lateral_limit = max_lateral_acceleration();

    // Accelerate forward along our heading
    let heading_dir = Vec2::new(heading().cos(), heading().sin());
    accelerate(heading_dir * forward_limit);

    // Stop ship translations by accelerating backwards against velocity
    if velocity().length() > 0.1 {
        accelerate(-velocity().normalize() * forward_limit);
    } else {
        accelerate(vec2(0.0, 0.0));
    }
}

// Orienting the ship (Turning)
pub fn show_rotation() {
    let max_ang_accel = max_angular_acceleration();
    let target_angle = 1.5; // ~90 degrees
    let difference = angle_diff(heading(), target_angle);
    
    // 1. High-level turning
    turn(difference * 5.0);

    // 2. Low-level Torque (PD Controller)
    let kp = 10.0;
    let kd = 2.0;
    let commanded_torque = kp * difference - kd * angular_velocity();
    torque(commanded_torque.clamp(-max_ang_accel, max_ang_accel));
}
```
