Quick Reference
Please file bugs at GitHub and give feedback on Discord or in-game. Also take a look at the wiki.
The API reference contains more detailed information.
Basics
Select a scenario from the list in the top-right of the page.
Click the run button in the editor to start the scenario with a new version of your code.
Controls
W/A/S/D: Pan the camera.
Space: Pause/resume.
N: Single-step (advance time by one tick and then pause).
F: Fast-forward.
M: Slow motion.
G: Show debug lines for all ships.
C: Chase, or follow the selected ship.
V: Toggle NLIPS, which makes smaller ships more visible when zoomed out.
B: Toggle postprocessing (blur).
Mouse wheel: Zoom.
Mouse click: Select a ship to show debugging info.
Ctrl-Enter: Execute
Ctrl-Shift-Enter: Replay
Ctrl-Alt-Enter: Replay and Pause
Language
Oort AIs are written in Rust. For an introduction to the language check out Rust By Example.

The starter code for each scenario includes a Ship struct with a tick method that the game will call 60 times per second. You can also store state in this struct which can be initialized in new and accessed with self.field_name.

All interactions between your AI and the game are done using the functions listed below. Many of these functions take or return Vec2, which is a 2-dimensional double-precision vector type.

Coordinate System
The game world is a 2D plane with the origin at the center. The X axis points to the right and the Y axis points up. The Wikipedia article on the Cartesian coordinate system has a picture.

The API uses units of meters, radians, and seconds.

Ship Status and Control
class() → Class: Returns the ship class.
position() → Vec2: Get the current position in meters.
velocity() → Vec2: Get the current velocity in m/s.
heading() → f64: Get the current heading in radians.
angular_velocity() → f64: Get the current angular velocity in radians/s.
health() → f64: Current health.
fuel() → f64: Current fuel (delta-v).
accelerate(acceleration: Vec2): Accelerate the ship. Units are m/s².
turn(speed: f64): Rotate the ship. Unit is radians/s.
torque(acceleration: f64): Angular acceleration. Unit is radians/s².
max_forward_acceleration() -> f64: Maximum forward acceleration.
max_backward_acceleration() -> f64: Maximum backward acceleration.
max_lateral_acceleration() -> f64: Maximum lateral acceleration.
max_angular_acceleration() -> f64: Maximum angular acceleration.
Weapons
fire(index: usize): Fire a weapon (gun or missile launcher).
aim(index: usize, angle: f64): Aim a weapon (for weapons on a turret).
explode(): Self-destruct.
Radar
set_radar_heading(angle: f64): Point the radar at the given heading.
radar_heading() -> f64: Get current radar heading.
set_radar_width(width: f64): Adjust the width of the radar beam (in radians).
radar_width() -> f64: Get current radar width.
scan() → Option<ScanResult>: Find an enemy ship illuminated by the radar.
struct ScanResult { position: Vec2, velocity: Vec2 }
Advanced Radar
set_radar_min_distance(dist: f64): Set the minimum distance filter.
radar_min_distance() -> f64: Get current minimum distance filter.
set_radar_max_distance(dist: f64): Set the maximum distance filter.
radar_max_distance() -> f64: Get current maximum distance filter.
set_radar_ecm_mode(mode: EcmMode): Set the Electronic Counter Measures (ECM) mode.
EcmMode::None: No ECM, radar will operate normally.
EcmMode::Noise: Decrease the enemy radar's signal to noise ratio, making it more difficult to detect targets and reducing accuracy of returned contacts.
select_radar(index: usize): Select the radar to control with subsequent API calls. Cruisers have two radars.
Radio
set_radio_channel(channel: usize): Change the radio channel (0 to 9). Takes effect next tick.
get_radio_channel() -> usize: Get the radio channel.
send(data: [f64; 4]): Send a message on a channel.
receive() -> Option<[f64; 4]>: Receive a message from the channel. The message with the strongest signal is returned.
send_bytes(data: &[u8]): Send a message on a channel as bytes, the data will be zero-filled or truncated to a length of 32 bytes.
receive_bytes() -> Option<[u8; 32]>: Just like receive, but instead the message will be returned as a byte array.
select_radio(index: usize): Select the radio to control with subsequent API calls. Frigates have 4 radios and cruisers have 8.
Special Abilities
activate_ability(ability: Ability): Activates a ship's special ability.
deactivate_ability(ability: Ability): Deactivates a ship's special ability.
active_abilities() → ActiveAbilities: Returns the ship's active abilities.
Available abilities:
Ability::Boost: Fighter and missile only. Applies a 100 m/s² forward acceleration for 2s. Reloads in 10s.
Ability::Decoy: Torpedo only. Mimics the radar signature of a Cruiser for 0.5s. Reloads in 10s.
Ability::Shield: Cruiser only. Deflects damage for 1s. Reloads in 5s.
Scalar Math
PI, TAU: Constants.
x.abs(): Absolute value.
x.sqrt(): Square root.
x.sin(), x.cos(), x.tan(): Trignometry.
See the Rust documentation for the full list of f64 methods.
Vector Math
For a refresher on vectors check out this tutorial.
vec2(x, y) → Vec2: Create a vector.
v.x, v.y → f64: Get a component of a vector.
v1 +- v2 → Vec2: Basic arithmetic between vectors.
v */ f64 → Vec2: Basic arithmetic between vectors and scalars.
-v → Vec2: Negate a vector.
v.length() → f64: Length.
v.normalize() → Vec2: Normalize to a unit vector.
v.rotate(f64) → Vec2: Rotate counter-clockwise.
v.angle() → f64: Angle of a vector.
v1.dot(v2: Vec2) → f64: Dot product.
v1.distance(v2: Vec2) → f64: Distance between two points.
Debugging
debug!(...): Add text to be displayed when the ship is selected by clicking on it. Works just like println!.
draw_line(v0: Vec2, v1: Vec2, color: u32): Draw a line visible when the ship is selected. Color is 24-bit RGB.
draw_triangle(center: Vec2, radius: f64, color: u32): Draw a triangle visible when the ship is selected.
draw_square(center: Vec2, radius: f64, color: u32): Draw a square visible when the ship is selected.
draw_diamond(center: Vec2, radius: f64, color: u32): Draw a diamond visible when the ship is selected.
draw_polygon(center: Vec2, radius: f64, sides: i32, angle: f64, color: u32): Draw a regular polygon visible when the ship is selected.
draw_text!(topleft: Vec2, color: u32, ...): Draw text. Works like println!.
Miscellaneous
current_tick() → u32: Returns the number of ticks elapsed since the simulation started.
current_time() → f64: Returns the number of seconds elapsed since the simulation started.
angle_diff(a: f64, b: f64) → f64: Returns the shortest (possibly negative) distance between two angles.
rand(low: f64, high: f64) → f64: Get a random number.
target() → Vec2: Used in some scenarios, returns the position of the target.
target_velocity() → Vec2: Used in some scenarios, returns the velocity of the target.
seed() → u128: Returns a seed useful for initializing a random number generator.
Extra Crates
The following crates are available for use in your code:

byteorder: Utilities to read and write binary data, useful for radio.
maths_rs: A linear algebra library.
oorandom: A random number generation library.
Ship Classes
Fighter: Small, fast, and lightly armored. One forward-facing gun and one missile launcher.
Frigate: Medium size with heavy armor. One forward-facing high-velocity gun, two turreted guns, and one missile launcher.
Cruiser: Large, slow, and heavily armored. One turreted heavy cannon, two missile launchers, and one torpedo launcher.
Missile: Highly maneuverable but unarmored. Explodes on contact or after an explode() call.
Torpedo: Better armor, larger warhead, but less maneuverable than a missile. Explodes on contact or after an explode() call.
