# Chapter 5: Math & Graphics Debugging

Building sophisticated AI behavior requires advanced geometry, relative trigonometry, and robust visual diagnostics. This chapter covers vector operations (`Vec2Extras`), circular math utilities, random state generation, text console logs, and overlay rendering.

For the corresponding checkable source code, see [math_and_debugging.rs](file:///home/jaguilar/projects/oort_ai/docs/examples/math_and_debugging.rs).

---

## 📐 Vector Algebra (`Vec2Extras`)

Oort uses the `maths-rs` library's `Vec2` type (which is `Vec2<f64>`). 

To simplify vector operations, Oort defines the **`Vec2Extras`** trait, which is implemented directly on `Vec2`. To use these methods, make sure you import `oort_api::prelude::*`.

### Vec2Extras Methods Reference

| Method | Return Type | Description |
|---|---|---|
| `length(self)` | `f64` | Returns the Euclidean length (magnitude) of the vector: $\sqrt{x^2 + y^2}$. |
| `normalize(self)` | `Vec2` | Returns a unit vector (length of `1.0`) pointing in the same direction. |
| `distance(self, other: Vec2)` | `f64` | Returns the distance between the two coordinate vectors. |
| `dot(self, other: Vec2)` | `f64` | Returns the dot product: $x_1x_2 + y_1y_2$. |
| `angle(self)` | `f64` | Returns the heading of the vector in radians (from `0.0` to `TAU`). |
| `rotate(self, angle: f64)` | `Vec2` | Rotates the vector counter-clockwise by `angle` radians. |

---

## 🔄 Angle & Trigonometry Utilities

Circular calculations can be tricky due to wrapping. Oort provides high-performance helpers to manage angles:

* **`PI`** & **`TAU`** (constants): Standard constants where $\text{TAU} = 2 \times \text{PI} \approx 6.283185$.
* **`angle_diff(a: f64, b: f64) -> f64`**
  * Computes the smallest angular delta between headings `a` and `b`.
  * Returns a value between $-\text{PI}$ and $+\text{PI}$.
  * A positive result represents a **counter-clockwise** rotation, and a negative result is a **clockwise** rotation.

---

## 🎲 Random Number Generator

Each match is initialized with a unique seed. Oort leverages this to provide a standard random number utility:

* **`rand(low: f64, high: f64) -> f64`**: Returns a pseudo-random `f64` in the range `[low, high)`.

---

## 🖥️ Screen Debugging & Console

You can print log strings to the ship console using the **`debug!`** macro.

* **`debug!("Format string: {}", arg)`**
  * Works exactly like standard `println!`.
  * Logs are visible in the Oort editor sidebar when you select your ship during a simulation playback.

---

## 🎨 In-Game Graphics Overlay

Oort allows you to draw shapes and text directly onto the space canvas to visualize target predictions, paths, or state flags. 

### Sizing and Limits
* **Maximums:** You can draw up to **`1024` line segments** and **`128` strings** per ship, per tick.
* **Colors:** All rendering functions accept colors as 24-bit integers. Use **`rgb(r: u8, g: u8, b: u8) -> u32`** to create a valid color value.

### Text Overlay
* **`draw_text!(topleft: Vec2, color: u32, "Format: {}", args)`**: Renders text floating in the game world.

### Shape Rendering APIs

| Function | Parameters | Description |
|---|---|---|
| `draw_line(a: Vec2, b: Vec2, color: u32)` | Point A, Point B, Color | Draws a line segment from A to B in world coordinates. |
| `draw_triangle(center: Vec2, radius: f64, color: u32)` | Center, Radius, Color | Draws an equilateral triangle centered at `center`. |
| `draw_square(center: Vec2, radius: f64, color: u32)` | Center, Radius, Color | Draws a square centered at `center`. |
| `draw_diamond(center: Vec2, radius: f64, color: u32)` | Center, Radius, Color | Draws a diamond centered at `center`. |
| `draw_polygon(center: Vec2, radius: f64, sides: i32, angle: f64, color: u32)` | Center, Radius, Sides, Rotation, Color | Draws a regular polygon with `sides` sides, rotated by `angle` radians. |

---

## 💻 Code Examples

Below is a compilable example illustrating vector algebra, circular helpers, random checks, and advanced overlay rendering.

```rust
use oort_api::prelude::*;

// Executing Vector Math operations
pub fn show_vector_math() {
    let position_a = vec2(100.0, 200.0);
    let position_b = vec2(150.0, 180.0);

    let offset = position_b - position_a;
    let dist: f64 = position_a.distance(position_b);
    let normalized_dir: Vec2 = offset.normalize();
    let angle_rad: f64 = offset.angle();

    // Rotate our vector by 90 degrees CCW (TAU / 4)
    let rotated_vec = offset.rotate(TAU / 4.0);

    debug!("Distance: {}, Angle: {}", dist, angle_rad);
    debug!("Sum: {:?}, Rotated: {:?}", offset, rotated_vec);
    debug!("Direction: {:?}", normalized_dir);
}

// Drawing overlays and debug vectors
pub fn show_graphics_debugging() {
    let my_pos = position();
    
    // Generate colors
    let red = rgb(255, 0, 0);
    let green = rgb(0, 255, 0);
    let blue = rgb(0, 0, 255);

    // Draw text indicator
    draw_text!(my_pos + vec2(0.0, 80.0), green, "Target Lock");

    // Draw prediction vector
    let prediction_point = my_pos + velocity() * 2.0; // Where we will be in 2 seconds
    draw_line(my_pos, prediction_point, red);

    // Draw shapes around the ship
    draw_square(my_pos, 40.0, blue);
    draw_triangle(my_pos, 30.0, green);
    draw_diamond(my_pos, 25.0, red);
    
    // Hexagon (6 sides) rotated by 45 degrees (TAU / 8)
    draw_polygon(my_pos, 50.0, 6, TAU / 8.0, blue);
}
```
