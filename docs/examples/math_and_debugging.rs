//! Compilable examples for the Oort Math, Vector, and Graphical Debugging APIs.
//! Covers Vec2 operations, angle utilities, random numbers, console debugs, and custom screen drawing.

use oort_api::prelude::*;

/// Example demonstrating the Vec2 type and the Vec2Extras trait.
pub fn show_vector_math() {
    // 1. Instantiating a vector
    // vec2(x, y) is a convenience function that constructs a maths-rs Vec2<f64>
    let v1: Vec2 = vec2(3.0, 4.0);
    let v2: Vec2 = vec2(1.0, 2.0);

    // 2. Vector addition, subtraction, scalar multiplication
    let sum = v1 + v2;
    let diff = v1 - v2;
    let scaled = v1 * 2.5;

    // 3. Methods provided by the Vec2Extras trait
    // length(): Returns the Euclidean length (magnitude) of the vector
    let mag: f64 = v1.length(); // 5.0

    // normalize(): Returns a vector of length 1 pointing in the same direction
    let norm: Vec2 = v1.normalize();

    // distance(): Returns the Euclidean distance between two coordinate vectors
    let dist: f64 = v1.distance(v2);

    // dot(): Returns the scalar dot product of two vectors
    let dot_prod: f64 = v1.dot(v2);

    // angle(): Returns the angle of the vector in radians (from 0 to TAU)
    let ang: f64 = v1.angle();

    // rotate(): Returns a copy of the vector rotated counter-clockwise by an angle in radians
    let rotated: Vec2 = v1.rotate(TAU / 4.0); // Rotate 90 degrees CCW

    debug!("V1 length: {}, norm: {:?}, dist to V2: {}", mag, norm, dist);
    debug!("Sum: {:?}, Diff: {:?}, Scaled: {:?}", sum, diff, scaled);
    debug!("Dot: {}, Angle: {}, Rotated: {:?}", dot_prod, ang, rotated);
}

/// Example demonstrating angle utilities, constants, and RNG.
pub fn show_math_utilities() {
    // Standard circular constants (TAU = 2 * PI)
    let tau_const: f64 = TAU;
    let pi_const: f64 = PI;

    // angle_diff(): Computes the shortest angular distance between two headings in radians.
    // A positive result represents a counter-clockwise rotation, and negative is clockwise.
    let heading_a = 0.5;
    let heading_b = 6.0; // Close to TAU
    let diff: f64 = angle_diff(heading_a, heading_b);

    // rand(): Returns a pseudo-random f64 in the range [low, high).
    // Automatically seeded by the Oort engine using the seed() value.
    let random_value: f64 = rand(10.0, 20.0);

    debug!("TAU: {}, PI: {}", tau_const, pi_const);
    debug!("Angle diff: {}", diff);
    debug!("Random number: {}", random_value);
}

/// Example demonstrating the console logging and in-game text/graphical debug tools.
pub fn show_graphics_debugging() {
    // 1. Text console logging
    // Writes text to the ship panel console when clicked in the editor
    debug!("Initializing diagnostics... Health is at {}%", health());

    // 2. Generating colors
    // rgb() returns a 24-bit integer color code from Red, Green, and Blue values [0-255]
    let color_red: u32 = rgb(255, 0, 0);
    let color_green: u32 = rgb(0, 255, 0);
    let color_blue: u32 = rgb(0, 0, 255);

    // 3. Drawing text overlays in the game world
    // draw_text! draws a floating string at a given world position [Vec2] and color [u32]
    // Up to 128 strings can be drawn per ship per tick.
    let text_pos = position() + vec2(0.0, 100.0);
    draw_text!(text_pos, color_green, "Telemetry Active");

    // 4. Drawing lines
    // Draws a line segment from point A to point B in world coordinates
    let line_start = position();
    let line_end = position() + vec2(100.0, 100.0);
    draw_line(line_start, line_end, color_red);

    // 5. Drawing pre-built shapes
    let center = position();
    let size = 50.0;
    
    // Triangle: center position [Vec2], radius [f64], color [u32]
    draw_triangle(center, size, color_blue);

    // Square: center position [Vec2], radius [f64], color [u32]
    draw_square(center, size, color_green);

    // Diamond: center position [Vec2], radius [f64], color [u32]
    draw_diamond(center, size, color_red);

    // General Regular Polygon: center [Vec2], radius [f64], sides [i32], rotation angle [f64], color [u32]
    let sides = 6; // Hexagon
    let rotation = 0.2;
    draw_polygon(center, size, sides, rotation, color_blue);
}
