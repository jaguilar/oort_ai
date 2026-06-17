use oort_api::prelude::*;
use crate::control::{quick_turn_with_target_omega, quick_turn};
use crate::missile::TargetTelemetry;
use crate::radar::{RadarController, DefaultScanSliceGenerator, Contact};
use crate::radio::SecureRadio;
use crate::physics::KinematicState;
use crate::aim::{AimAt, GunAimer};

fn cluster_contacts(contacts: &[Contact]) -> Vec<Vec<Contact>> {
    if contacts.is_empty() {
        return Vec::new();
    }
    if contacts.len() <= 3 {
        return contacts.iter().map(|c| vec![c.clone()]).collect();
    }
    let mut clusters: Vec<Vec<Contact>> = contacts.iter().map(|c| vec![c.clone()]).collect();
    while clusters.len() > 3 {
        let mut min_dist = f64::MAX;
        let mut merge_indices = (0, 0);
        for i in 0..clusters.len() {
            for j in (i + 1)..clusters.len() {
                for c1 in &clusters[i] {
                    for c2 in &clusters[j] {
                        let dist = c1.current_position().distance(c2.current_position());
                        if dist < min_dist {
                            min_dist = dist;
                            merge_indices = (i, j);
                        }
                    }
                }
            }
        }
        let (i, j) = merge_indices;
        let mut target_cluster = clusters[j].clone();
        clusters[i].append(&mut target_cluster);
        clusters.swap_remove(j);
    }
    clusters
}

struct ExpectedIntercept {
    position: Vec2,
    expiry_tick: u32,
}
const GUN_AIMER: GunAimer = GunAimer::new(Vec2 { x: 0.0, y: 0.0 }, 2000.0);

pub struct Cruiser {
    radar_controller: RadarController,
    weapon_targets: [Option<u32>; 4], // Targets assigned to weapon slots 0, 1, and 2
    intercept_plots: Vec<ExpectedIntercept>, // Intercept points to draw for active bullets
    
    seen_target_ids: Vec<u32>,
    
    missile_sender: crate::missile::MissileRadioSender,

    // NEW FIELDS FOR FIGHTER COORDINATION AND ORBIT BEHAVIOR
    fighter_radio: SecureRadio,
    fighter_targets_to_broadcast: Option<Vec<TargetTelemetry>>,
    fighter_broadcast_index: usize,
}

impl Cruiser {
    pub fn new(missile_radio: SecureRadio, fighter_radio: SecureRadio) -> Cruiser {
        let mut rc = RadarController::new();
        // Double the base scan range from 10000.0 to 20000.0
        rc.slice_generator = Box::new(DefaultScanSliceGenerator::new(0.6, 20000.0));

        Cruiser {
            radar_controller: rc,
            weapon_targets: [None; 4],
            intercept_plots: Vec::new(),
            seen_target_ids: Vec::new(),
            missile_sender: crate::missile::MissileRadioSender::new(missile_radio),
            fighter_radio,
            fighter_targets_to_broadcast: None,
            fighter_broadcast_index: 0,
        }
    }

    /// Selects a missile target from `fighters`. Contacts satisfying `in_priority_set` are
    /// preferred; if none qualify, a random contact from the full list is returned instead.
    fn select_missile_target<'a, F>(
        &self,
        fighters: &[&'a crate::radar::Contact],
        in_priority_set: F,
    ) -> Option<u32>
    where
        F: Fn(&crate::radar::Contact) -> bool,
    {
        let priority: Vec<&&crate::radar::Contact> = fighters.iter()
            .filter(|f| in_priority_set(f))
            .collect();

        let pool: Vec<&&crate::radar::Contact> = if !priority.is_empty() {
            priority
        } else {
            fighters.iter().collect()
        };
        
        // Draw green squares around all targets that were in the pool.
        for f in pool.iter() {
            draw_square(vec2(f.current_position().x as f64, f.current_position().y as f64), 300.0, rgb(0, 255, 0));
        }

        if pool.is_empty() {
            return None;
        }
        let idx = (rand(0.0, pool.len() as f64).floor() as usize).min(pool.len() - 1);
        Some(pool[idx].id)
    }

    fn num_guns_targeting(&self, target_id: u32, exclude_weapon: Option<usize>) -> usize {
        (0..3)
            .filter(|&w| Some(w) != exclude_weapon && self.weapon_targets[w] == Some(target_id))
            .count()
    }

    fn is_better_target(&self, w: usize, a: &crate::radar::Contact, b: &crate::radar::Contact) -> bool {
        let total_targeting_a = self.num_guns_targeting(a.id, Some(w));
        let total_targeting_b = self.num_guns_targeting(b.id, Some(w));

        let is_clean_a = total_targeting_a == 0;
        let is_clean_b = total_targeting_b == 0;

        if is_clean_a != is_clean_b {
            return is_clean_a;
        }

        if total_targeting_a != total_targeting_b {
            return total_targeting_a < total_targeting_b;
        }

        let ship_pos = position();
        match w {
            0 => {
                // Gun 0: Turreted heavy cannon. Targets closest fighter (same as frigate's turreted guns).
                let dist_a = ship_pos.distance(a.current_position());
                let dist_b = ship_pos.distance(b.current_position());
                dist_a < dist_b
            }
            1 => {
                // Gun 1: Left missile launcher. Prefers targets on the left side.
                let angle_a = (a.current_position() - ship_pos).angle();
                let angle_b = (b.current_position() - ship_pos).angle();
                let diff_a = angle_diff(heading(), angle_a);
                let diff_b = angle_diff(heading(), angle_b);
                let a_on_left = diff_a >= 0.0;
                let b_on_left = diff_b >= 0.0;
                if a_on_left != b_on_left {
                    a_on_left
                } else {
                    let dist_a = ship_pos.distance(a.current_position());
                    let dist_b = ship_pos.distance(b.current_position());
                    dist_a < dist_b
                }
            }
            2 => {
                // Gun 2: Right missile launcher. Prefers targets on the right side.
                let angle_a = (a.current_position() - ship_pos).angle();
                let angle_b = (b.current_position() - ship_pos).angle();
                let diff_a = angle_diff(heading(), angle_a);
                let diff_b = angle_diff(heading(), angle_b);
                let a_on_right = diff_a < 0.0;
                let b_on_right = diff_b < 0.0;
                if a_on_right != b_on_right {
                    a_on_right
                } else {
                    let dist_a = ship_pos.distance(a.current_position());
                    let dist_b = ship_pos.distance(b.current_position());
                    dist_a < dist_b
                }
            }
            _ => false,
        }
    }

    pub fn tick(&mut self) {
        let current_tick = current_tick();

        // 0. Update priority targets list based on weapon targeting and reload status
        let mut priority_ids = Vec::new();
        for w in 0..3 {
            if let Some(tid) = self.weapon_targets[w] {
                priority_ids.push(tid);
            }
        }
        self.radar_controller.priority_target_frequencies = priority_ids
            .into_iter()
            .map(|id| (id, 6.0 * TICK_LENGTH))
            .collect();
        self.radar_controller.update();
        self.missile_sender.send_missile_contact(self.radar_controller.contacts());

        // 2. Fetch active fighter contacts from the radar controller
        let contacts = self.radar_controller.contacts().to_vec();
        let fighters: Vec<crate::radar::Contact> = contacts.iter()
            .filter(|c| c.class != Class::Missile)
            .cloned()
            .collect();
        let fighter_refs: Vec<&crate::radar::Contact> = fighters.iter().collect();

        for f in &fighters {
            if !self.seen_target_ids.contains(&f.id) {
                self.seen_target_ids.push(f.id);
            }
        }

        // Cruiser target clustering and broadcasting
        if self.fighter_targets_to_broadcast.is_none() && self.fighter_broadcast_index == 0 {
            let active_fighters: Vec<Contact> = fighters.clone();
            let clusters = cluster_contacts(&active_fighters);
            
            if current_tick % 10 == 0 && !active_fighters.is_empty() {
                let cluster_sizes: Vec<usize> = clusters.iter().map(|c| c.len()).collect();
                debug!("Cruiser clustering check: active_fighters count={}, clusters count={}, cluster_sizes={:?}", active_fighters.len(), clusters.len(), cluster_sizes);
            }

            if clusters.len() == 3 && clusters[0].len() >= 2 && clusters[1].len() >= 2 && clusters[2].len() >= 2 {
                let t0 = TargetTelemetry {
                    position: clusters[0][0].current_position(),
                    velocity: clusters[0][0].current_velocity(),
                    rssi: 0.0,
                    class: clusters[0][0].class,
                    tick: current_tick as u8,
                };
                let t1 = TargetTelemetry {
                    position: clusters[1][0].current_position(),
                    velocity: clusters[1][0].current_velocity(),
                    rssi: 0.0,
                    class: clusters[1][0].class,
                    tick: current_tick as u8,
                };
                let t2 = TargetTelemetry {
                    position: clusters[2][0].current_position(),
                    velocity: clusters[2][0].current_velocity(),
                    rssi: 0.0,
                    class: clusters[2][0].class,
                    tick: current_tick as u8,
                };
                self.fighter_targets_to_broadcast = Some(vec![t0, t1, t2]);
                self.fighter_broadcast_index = 0;
                debug!("Cruiser detected 3 distinct enemy clusters! Staging broadcasts.");
            }
        }

        if let Some(ref targets) = self.fighter_targets_to_broadcast {
            if self.fighter_broadcast_index < 3 {
                let telemetry = targets[self.fighter_broadcast_index];
                self.fighter_radio.transmit(telemetry.serialize());
                let ch = self.fighter_radio.channel_offset;
                debug!("Cruiser broadcast target {} on fighter channel (slot 1, offset {}): pos={:?}, vel={:?}, tick={}", self.fighter_broadcast_index, ch, telemetry.position, telemetry.velocity, telemetry.tick);
                self.fighter_broadcast_index += 1;
            }
            if self.fighter_broadcast_index == 3 {
                self.fighter_targets_to_broadcast = None;
            }
        }

        // 3. Update Weapon Assignments
        // Prune targets that are no longer tracked/alive
        for w in 0..3 {
            if let Some(tid) = self.weapon_targets[w] {
                if !fighters.iter().any(|f| f.id == tid) {
                    self.weapon_targets[w] = None;
                }
            }
        }

        // Build list of weapons that are ready and need assignment (currently have None target)
        let mut assignment_order = Vec::new();
        if self.weapon_targets[0].is_none() {
            assignment_order.push(0);
        }
        if self.weapon_targets[1].is_none() && reload_ticks(1) <= 4 {
            assignment_order.push(1);
        }
        if self.weapon_targets[2].is_none() && reload_ticks(2) <= 4 {
            assignment_order.push(2);
        }

        for &w in &assignment_order {
            if w == 0 {
                let mut best_fighter: Option<&crate::radar::Contact> = None;
                for f in &fighters {
                    // If we've seen less than 3 targets, do not allow duplicate target assignments.
                    if self.seen_target_ids.len() < 3 {
                        let total_targeting = self.num_guns_targeting(f.id, Some(w));
                        if total_targeting > 0 {
                            continue;
                        }
                    }

                    if let Some(best) = best_fighter {
                        if self.is_better_target(w, f, best) {
                            best_fighter = Some(f);
                        }
                    } else {
                        best_fighter = Some(f);
                    }
                }
                if let Some(f) = best_fighter {
                    self.weapon_targets[w] = Some(f.id);
                }
            }
        }

        let is_near_angle = |f: &crate::radar::Contact, angle: f64| {
            let dir = f.current_position() - position();
            angle_diff(angle, dir.angle()).abs() <= 67.5f64.to_radians()
        };
        if reload_ticks(1) == 0 {
            let left_angle = heading() + std::f64::consts::FRAC_PI_2;
            debug!("considering left missile targets");
            self.weapon_targets[1] = self.select_missile_target(&fighter_refs, |f| is_near_angle(f, left_angle));
        } else if reload_ticks(2) == 0 {
            debug!("considering right missile targets");
            let right_angle = heading() - std::f64::consts::FRAC_PI_2;
            self.weapon_targets[2] = self.select_missile_target(&fighter_refs, |f| is_near_angle(f, right_angle));
        }

        // 4. Weapon Aiming and Firing
        // Gun 0: Turreted heavy cannon (Bullet Speed: 2000.0 m/s)
        if let Some(tid) = self.weapon_targets[0] {
            if let Some(f) = fighters.iter().find(|fighter| fighter.id == tid) {
                let us_state = KinematicState::self_state();
                let aim_res = GUN_AIMER.aim_at(&f.kinematic, &us_state);

                if let Some((lead_dir, omega)) = aim_res {
                    let lead_angle = lead_dir.angle();
                    aim(0, lead_angle);

                    // Rotate the ship to face the target generally to facilitate radar sweep/movement
                    quick_turn_with_target_omega(lead_angle, omega);

                    // Visualization
                    let time_to_impact = (f.current_position() - position()).length() / GUN_AIMER.bullet_speed;
                    let p_e = f.position_at(current_tick + (time_to_impact / TICK_LENGTH).round() as u32);
                    draw_line(position(), p_e, rgb(255, 0, 0));
                    draw_square(p_e, 25.0, rgb(255, 0, 0));

                    if reload_ticks(0) == 0 {
                        fire(0);
                        let expiry_tick = current_tick + (time_to_impact / TICK_LENGTH).round() as u32;
                        self.intercept_plots.push(ExpectedIntercept {
                            position: p_e,
                            expiry_tick,
                        });
                    }
                } else {
                    let direct_angle = (f.current_position() - position()).angle();
                    aim(0, direct_angle);
                    quick_turn(direct_angle);
                }
            }
        } else {
            // Turn ship towards fallback target
            let fallback_target = self.weapon_targets[1]
                .or(self.weapon_targets[2])
                .and_then(|tid| fighters.iter().find(|f| f.id == tid));
            if let Some(f) = fallback_target {
                let direct_angle = (f.current_position() - position()).angle();
                quick_turn(direct_angle);
            }
        }

        if self.missile_sender.missile_radio.avail() {
            let mut launched_slot = None;
            if self.weapon_targets[1].is_some() && reload_ticks(1) == 0 {
                launched_slot = Some(1);
            } else if self.weapon_targets[2].is_some() && reload_ticks(2) == 0 {
                launched_slot = Some(2);
            }

            if let Some(w) = launched_slot {
                if let Some(tid) = self.weapon_targets[w] {
                    self.missile_sender.fire_missile(w, tid);
                    if let Some(f) = fighters.iter().find(|fighter| fighter.id == tid) {
                        draw_diamond(f.current_position(), 300.0, rgb(78, 188, 44));
                    }
                    
                    // Clear assignment immediately to target a different enemy on subsequent launch
                    self.weapon_targets[w] = None;
                }
            }
        }

        // 5. Draw expected intercept plots for debug
        self.intercept_plots.retain(|plot| current_tick <= plot.expiry_tick);
        for plot in &self.intercept_plots {
            draw_polygon(plot.position, 8.0, 8, 0.0, rgb(255, 0, 0));
        }

        // 6. Draw a blue triangle at each contact's estimated position
        for contact in contacts {
            draw_triangle(contact.current_position(), 15.0, rgb(0, 0, 255));
        }
    }
}
