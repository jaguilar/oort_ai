//! Compilable examples for the Oort Combat, Weapons, and Abilities APIs.
//! Covers weapon aiming/firing, reloading, abilities activation, and ship class stats.

use oort_api::prelude::*;
use oort_api::ActiveAbilities;

/// Example of aiming and firing turreted weapons.
pub fn show_weapons_firing() {
    let weapon_slot = 0; // Oort supports multiple weapon/launcher slots (0 to 3)

    // 1. Aim the turreted weapon at a target heading (e.g. 1.0 radians)
    let target_heading = 1.0;
    aim(weapon_slot, target_heading);

    // 2. Query weapon status (reload time in ticks; 0 means ready)
    let ticks_remaining: u32 = reload_ticks(weapon_slot);

    if ticks_remaining == 0 {
        // Fire the weapon in slot 0
        fire(weapon_slot);
        debug!("Weapon slot {} fired!", weapon_slot);
    } else {
        debug!("Weapon slot {} is reloading ({} ticks left)", weapon_slot, ticks_remaining);
    }
}

/// Example of managing special abilities (Boost, Shield, Decoy) depending on the ship class.
pub fn show_abilities() {
    let my_class = class();

    match my_class {
        Class::Fighter => {
            // Fighters have the Boost ability (applies 100 m/s² forward acceleration for 2s)
            // It has a 10s cooldown.
            activate_ability(Ability::Boost);
            
            // Check if Boost is currently active
            let active: ActiveAbilities = active_abilities();
            if active.get_ability(Ability::Boost) {
                debug!("Fighter boost is active!");
            }
        }
        Class::Cruiser => {
            // Cruisers have the Shield ability (deflects projectiles for 1s)
            // It has a 5s cooldown.
            activate_ability(Ability::Shield);
            
            // We can also deactivate abilities manually if needed
            let active = active_abilities();
            if active.get_ability(Ability::Shield) {
                debug!("Cruiser shield is active; deactivating!");
                deactivate_ability(Ability::Shield);
            }
        }
        Class::Torpedo => {
            // Torpedoes have the Decoy ability (mimics the radar signature of a Cruiser for 0.5s)
            // It has a 10s cooldown.
            activate_ability(Ability::Decoy);
        }
        _ => {
            // Other classes or entities may not have active abilities
            debug!("No abilities for class {:?}", my_class);
        }
    }

    // Inspecting all currently active abilities on the ship
    let current_active: ActiveAbilities = active_abilities();
    for ability in current_active.active_iter() {
        debug!("Active ability: {:?}", ability);
    }
}

/// Example showing self-destruction (commonly used by missiles).
pub fn show_self_destruct() {
    // If the entity is a missile, explode when close to target (less than 10 meters)
    if class() == Class::Missile {
        let dist_to_target = position().distance(target());
        if dist_to_target < 10.0 {
            debug!("Target reached! Exploding!");
            // Self-destruct producing a damaging blast
            explode();
        }
    }
}
