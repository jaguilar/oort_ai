# Chapter 3: Radar & Targeting Systems

Tracking moving targets and navigating space fields in Oort is driven by the ship's dual active radar systems. This chapter covers selecting radars, adjusting headings and sweep widths, distance clipping, reading contacts, noise jamming (ECM), and tutorial-specific shortcut functions.

For the corresponding checkable source code, see [radar_and_targeting.rs](file:///home/jaguilar/projects/oort_ai/docs/examples/radar_and_targeting.rs).

---

## 📡 The Dual Radar System

Ships are equipped with two independent radar systems (index `0` and `1`). 

### Selecting the Radar
Before calling any radar control or scanning function, you **must select which radar** subsequent commands will apply to:
* **`select_radar(index: usize)`**: Activates radar `0` or `1` for subsequent command queries.

---

## ⚙️ Radar Configuration APIs

Each radar can be configured with specific directions, sweep fields, and range filters.

| Function | Get / Set | Description |
|---|---|---|
| `radar_heading()` / `set_radar_heading(heading: f64)` | Both | The center heading of the radar beam in absolute world radians. |
| `radar_width()` / `set_radar_width(width: f64)` | Both | The field of view sweep width in radians. |
| `radar_min_distance()` / `set_radar_min_distance(dist: f64)` | Both | The minimum range filter in meters. Contacts closer than this are ignored. |
| `radar_max_distance()` / `set_radar_max_distance(dist: f64)` | Both | The maximum range filter in meters. Contacts further than this are ignored. |

> [!TIP]
> **Beam Convergence:** Setting `set_radar_width(0.0)` creates an infinitely narrow beam. This maximizes signal-to-noise ratio (SNR) for a locked target but makes it extremely easy to lose the target if they maneuver out of the line of sight.

---

## 🔍 Contact Scanning (`ScanResult`)

Calling **`scan() -> Option<ScanResult>`** searches the configured sector for contacts. It returns the contact with the **highest received signal strength** (RSSI) within the sweep area.

### ScanResult Structure

```rust
pub struct ScanResult {
    pub class: Class,      // Class of the contact (Fighter, Cruiser, Torpedo, Asteroid, etc)
    pub position: Vec2,    // Approximate coordinates (in meters)
    pub velocity: Vec2,    // Approximate velocity (in m/s)
    pub rssi: f64,         // Received Signal Strength Indicator in dBm
    pub snr: f64,          // Signal-to-Noise Ratio in dB
}
```

---

## ⚡ Electronic Countermeasures (ECM)

* **`radar_ecm_mode()`** / **`set_radar_ecm_mode(mode: EcmMode)`**
  * Configures the selected radar's electronic warfare suite.
  * **`EcmMode::None`**: Normal radar behavior.
  * **`EcmMode::Noise`**: Emits a jamming signal that lowers the Signal-to-Noise Ratio (SNR) of any enemy radars swept by your beam, making it difficult for them to track or scan your ship.

---

## 🎯 Tutorial Targeting Helpers

In tutorial scenarios, Oort provides two globally available target telemetry helper functions (no radar scanning required):

* **`target() -> Vec2`**: Returns the absolute position of the scenario target in meters.
* **`target_velocity() -> Vec2`**: Returns the absolute velocity of the scenario target in m/s.

---

## 💻 Code Examples

Below is a compilable workflow showing how to sweep the space, lock onto an active contact, activate ECM noise jamming, and query tutorial parameters.

```rust
use oort_api::prelude::*;

// Sweeping and tracking target contacts
pub fn show_radar_scanning() {
    // Select radar slot 0
    select_radar(0);
    
    let contact = scan();
    match contact {
        Some(result) => {
            // Target found! Point our radar tightly at them
            let offset_vector = result.position - position();
            let angle_to_target = offset_vector.angle();
            
            set_radar_heading(angle_to_target);
            set_radar_width(0.01); // Narrow tracking beam
            
            debug!("Target locked! Position: {:?}", result.position);
        }
        None => {
            // Sweep wide to locate targets
            let sweep_head = radar_heading() + 0.1;
            set_radar_heading(sweep_head);
            set_radar_width(0.5); // Wide search beam
        }
    }
}

// Activating ECM Noise Jamming
pub fn show_radar_ecm() {
    select_radar(0);
    
    // Check mode and set to active Noise jamming
    if radar_ecm_mode() == EcmMode::None {
        set_radar_ecm_mode(EcmMode::Noise);
        debug!("ECM Jammer Engaged");
    }
}

// Proximity tracking in tutorials
pub fn show_tutorial_helpers() {
    let t_pos = target();
    let t_vel = target_velocity();
    let distance = position().distance(t_pos);
    
    debug!("Tutorial Target Distance: {}m, Velocity: {:?}", distance, t_vel);
}
```
