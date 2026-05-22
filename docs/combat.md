# Chapter 2: Weapons, Classes & Abilities

Engaging targets and surviving combat encounters in Oort requires managing the ship's weapons systems, leveraging class-specific active abilities, and (for missiles) handling self-destruction.

For the corresponding checkable source code, see [combat.rs](file:///home/jaguilar/projects/oort_ai/docs/examples/combat.rs).

---

## 🔫 Weapons Control API

Ships can have up to four weapons or launcher slots (indexed from `0` to `3`). Each weapon is independent and can be aimed and fired individually.

| Function | Description |
|---|---|
| `aim(index: usize, heading: f64)` | Points the turret/launcher at `index` to a target angle in absolute world radians. |
| `fire(index: usize)` | Fires the weapon at `index` in the next tick. |
| `reload_ticks(index: usize) -> u32` | Returns the remaining reload cooldown in ticks. `0` means the weapon is ready to fire. |

> [!NOTE]
> Turret rotation is not instantaneous. The turret rotates toward the commanded heading over multiple ticks. To achieve maximum accuracy, ensure your turret has finished aligning before calling `fire()`.

---

## 🛡️ Ship Classes & Stats

Oort includes multiple ship classes, each with unique physical parameters, masses, and weapon layouts. You can retrieve these base stats at runtime by calling `class().default_stats()`.

### Default Class Stats Overview

| Class | Max Health | Mass (kg) | Max Forward Accel (m/s²) | Special Active Ability |
|---|---|---|---|---|
| `Fighter` | 100 | 15,000 | 60.0 | `Ability::Boost` |
| `Frigate` | 10,000 | 4,000,000 | 10.0 | None |
| `Cruiser` | 20,000 | 9,000,000 | 5.0 | `Ability::Shield` |
| `Torpedo` | 100 | 500 | 70.0 | `Ability::Decoy` |
| `Missile` | 20 | 150 | 300.0 | None (Uses `explode()`) |

---

## ⚡ Active Abilities API

Certain ship classes can activate powerful temporary special abilities:

* **`Ability::Boost`** (Fighter & Missile): Applies an extra `100 m/s²` forward acceleration for `2.0` seconds. Recharges in `10.0` seconds.
* **`Ability::Shield`** (Cruiser only): Deflects all incoming hostile projectiles for `1.0` second. Recharges in `5.0` seconds.
* **`Ability::Decoy`** (Torpedo only): Mimics the radar signature of a Cruiser for `0.5` seconds to draw fire or fool scans. Recharges in `10.0` seconds.

### Control Functions

* **`activate_ability(ability: Ability)`**: Activates the specified ability for the current tick.
* **`deactivate_ability(ability: Ability)`**: Deactivates the specified ability.
* **`active_abilities() -> ActiveAbilities`**: Queries the list of currently active abilities.

### ActiveAbilities Methods

The returned `ActiveAbilities` struct provides several methods:
* `get_ability(ability: Ability) -> bool`: Checks if a specific ability is active.
* `active_iter()`: Returns an iterator over all currently active abilities.

---

## 💥 Self-Destruction

* **`explode()`**
  * Immediately triggers a self-destruct sequence, destroying the ship and creating a localized, damaging explosion.
  * This is commonly used by **`Class::Missile`** and **`Class::Torpedo`** entities when they get close enough to their target.

---

## 💻 Code Examples

Below is a compilable example illustrating how to coordinate aiming, reload tracking, class-specific abilities, and self-destruction.

```rust
use oort_api::prelude::*;
use oort_api::ActiveAbilities;

// Standard turret aiming and firing sequence
pub fn show_weapons_firing() {
    let weapon_slot = 0;
    let target_heading = 1.0;

    // Aim turret in world space
    aim(weapon_slot, target_heading);

    // Only fire if the weapon is ready
    if reload_ticks(weapon_slot) == 0 {
        fire(weapon_slot);
        debug!("Weapon slot {} fired!", weapon_slot);
    } else {
        debug!("Weapon reloading...");
    }
}

// Utilizing class abilities
pub fn show_abilities() {
    let my_class = class();

    match my_class {
        Class::Fighter => {
            // Engage forward boost thrust
            activate_ability(Ability::Boost);
            
            let active: ActiveAbilities = active_abilities();
            if active.get_ability(Ability::Boost) {
                debug!("Fighter boost is active!");
            }
        }
        Class::Cruiser => {
            // Activate shield deflection
            activate_ability(Ability::Shield);
        }
        _ => {}
    }
}

// Exploding on target proximity (Missile logic)
pub fn show_self_destruct() {
    if class() == Class::Missile {
        let dist = position().distance(target());
        if dist < 10.0 {
            // Explode close to target
            explode();
        }
    }
}
```
