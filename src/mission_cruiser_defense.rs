use oort_api::prelude::*;
use std::rc::Rc;
use std::cell::RefCell;
use crate::missile::MissileGuidance;
use crate::radio::{SecureRadio, RadioManager};
use crate::fighter::Fighter;
use crate::cruiser::Cruiser;

enum ShipState {
    Missile(MissileGuidance),
    Fighter(Fighter),
    Cruiser(Cruiser),
}

pub struct Ship {
    state: ShipState,
}

impl Ship {
    pub fn new() -> Ship {
        let radio_manager = Rc::new(RefCell::new(RadioManager::new()));
        let missile_radio = SecureRadio::new(1337, 0, radio_manager.clone());
        let fighter_radio = SecureRadio::new(1338, 4, radio_manager);

        let state = if class() == Class::Missile {
            let mut mg = MissileGuidance::new();
            mg.target_channel = 3;
            mg.secure_radio = Some(missile_radio);
            ShipState::Missile(mg)
        } else if class() == Class::Fighter {
            ShipState::Fighter(Fighter::new(missile_radio, fighter_radio))
        } else {
            ShipState::Cruiser(Cruiser::new(missile_radio, fighter_radio))
        };

        Ship { state }
    }

    pub fn tick(&mut self) {
        match &mut self.state {
            ShipState::Missile(mg) => mg.tick(),
            ShipState::Fighter(fighter) => fighter.tick(),
            ShipState::Cruiser(cruiser) => cruiser.tick(),
        }
    }
}
