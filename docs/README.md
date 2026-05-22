# Oort API Reference Documentation

Welcome to the **Oort API Reference**. Oort is a 2D space simulation environment where you program a ship in Rust to complete various challenges, from simple combat tutorials to complex multiplayer space battle matches.

This documentation serves as a comprehensive, reliable guide to help you build smart, responsive, and robust autopilot bots. 

---

## 📖 Table of Contents

The documentation is organized into five modular chapters:

1. **[Ship Physics & Control](file:///home/jaguilar/projects/oort_ai/docs/ship_control.md)**
   * Navigation, speed limits, positioning, and flight dynamics.
   * Compilable examples: [ship_control.rs](file:///home/jaguilar/projects/oort_ai/docs/examples/ship_control.rs).
2. **[Weapons, Classes & Abilities](file:///home/jaguilar/projects/oort_ai/docs/combat.md)**
   * Weapon aiming and firing, ship class stats, special abilities (Boost, Shield, Decoy), and self-destruction.
   * Compilable examples: [combat.rs](file:///home/jaguilar/projects/oort_ai/docs/examples/combat.rs).
3. **[Radar & Targeting Systems](file:///home/jaguilar/projects/oort_ai/docs/radar_and_targeting.md)**
   * Multi-radar systems, sweep angle width, contact scanning (`ScanResult`), Electronic Countermeasures (ECM), and tutorial helpers.
   * Compilable examples: [radar_and_targeting.rs](file:///home/jaguilar/projects/oort_ai/docs/examples/radar_and_targeting.rs).
4. **[Inter-Ship Communications](file:///home/jaguilar/projects/oort_ai/docs/communication.md)**
   * Broadcaster/Receiver radio networks, float `Message` transfers (`[f64; 4]`), and raw `[u8; 32]` packet serialization.
   * Compilable examples: [communication.rs](file:///home/jaguilar/projects/oort_ai/docs/examples/communication.rs).
5. **[Math Utilities & Visual Diagnostics](file:///home/jaguilar/projects/oort_ai/docs/math_and_debugging.md)**
   * 2D vector math (`Vec2Extras` trait), angle arithmetic, random numbers, debug console logs (`debug!`), and floating graphical overlays.
   * Compilable examples: [math_and_debugging.rs](file:///home/jaguilar/projects/oort_ai/docs/examples/math_and_debugging.rs).

---

## 🛠️ Verification and Compilable Examples

A major challenge with AI-generated documentation is maintaining accuracy as APIs evolve. To prevent drift, this documentation suite is backed by **fully compilable Rust source files** located under the [docs/examples/](file:///home/jaguilar/projects/oort_ai/docs/examples) directory:

- [ship_control.rs](file:///home/jaguilar/projects/oort_ai/docs/examples/ship_control.rs)
- [combat.rs](file:///home/jaguilar/projects/oort_ai/docs/examples/combat.rs)
- [radar_and_targeting.rs](file:///home/jaguilar/projects/oort_ai/docs/examples/radar_and_targeting.rs)
- [communication.rs](file:///home/jaguilar/projects/oort_ai/docs/examples/communication.rs)
- [math_and_debugging.rs](file:///home/jaguilar/projects/oort_ai/docs/examples/math_and_debugging.rs)

These examples are registered directly in the crate's `src/lib.rs` file. This means that:
* Every time you run **`cargo check`** or **`cargo build`**, Cargo automatically compiles and validates every single example block in the documentation.
* The examples are fully modularized so we can load and edit them selectively, minimizing context token overhead.

### Validating the Examples

To verify that the documentation and examples are completely correct and compilation-sound, simply run:

```bash
cargo check
```

If it compiles with no errors, you are guaranteed that all Oort API functions, enums, structures, and types referenced in the documentation are 100% accurate and up to date!
