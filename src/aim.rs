use oort_api::prelude::*;
use crate::physics::KinematicState;

pub trait AimAt {
    fn aim_at(
        &self,
        target: &KinematicState,
        us: &KinematicState,
    ) -> Option<(Vec2, f64)>;
}

#[derive(Clone, Debug)]
pub struct DebugIntercept {
    pub pos: Vec2,
    pub actual_pos: Vec2,
    pub intercept_tick: u32,
}

#[derive(Clone, Debug)]
pub struct FiringSolution {
    pub aim_dir: Vec2,
    pub omega: f64,
    pub intercept_pos: Vec2,
    pub intercept_time: f64,
}

pub struct FireControl {
    pub gun_id: usize,
    pub magazine_size: u32,
    pub bullets_in_magazine: u32,
    pub reload_ticks_fn: fn(usize) -> u32,
    pub aimer: Box<dyn AimAt>,
    pub debug_intercepts: Vec<DebugIntercept>,
}

impl FireControl {
    pub fn from_gun(id: i32) -> Self {
        Self::new(id, reload_ticks)
    }

    pub fn new(id: i32, reload_ticks_fn: fn(usize) -> u32) -> Self {
        let aimer: Box<dyn AimAt> = match id {
            0 => Box::new(GunAimer::new(Vec2::new(20.0, 0.0), 1000.0)),
            1 => Box::new(crate::missile::MissileAimer::new(1400.0, 250.0)),
            _ => panic!("Unsupported gun id: {}", id),
        };
        match id {
            0 => Self {
                gun_id: 0,
                magazine_size: 30,
                bullets_in_magazine: 30,
                reload_ticks_fn,
                aimer,
                debug_intercepts: Vec::new(),
            },
            1 => Self {
                gun_id: 1,
                magazine_size: 1,
                bullets_in_magazine: 1,
                reload_ticks_fn,
                aimer,
                debug_intercepts: Vec::new(),
            },
            _ => panic!("Unsupported gun id: {}", id),
        }
    }

    pub fn fire(&mut self) {
        if (self.reload_ticks_fn)(self.gun_id) != 0 {
            return;
        }
        fire(self.gun_id);
        let current_bullets = self.bullets_in_magazine();
        self.bullets_in_magazine = current_bullets.saturating_sub(1);
    }

    pub fn reload_time(&self) -> f64 {
        ((self.reload_ticks_fn)(self.gun_id) as f64) * TICK_LENGTH
    }

    pub fn bullets_in_magazine(&self) -> u32 {
        let r_ticks = (self.reload_ticks_fn)(self.gun_id);
        if self.bullets_in_magazine == 0 {
            if r_ticks > 0 {
                0
            } else {
                self.magazine_size
            }
        } else {
            self.bullets_in_magazine
        }
    }

    pub fn solve_aim(&self, target: &KinematicState, us: &KinematicState) -> Option<FiringSolution> {
        let (aim_dir, omega) = self.aimer.aim_at(target, us)?;
        let (intercept_pos, intercept_time) = match self.gun_id {
            0 => {
                let us_heading = us.heading.unwrap_or(0.0);
                let gun_pos = us.position + Vec2::new(20.0, 0.0).rotate(us_heading);
                let bullet_speed = 1000.0;
                let dp0 = target.position - gun_pos;
                let r_len = dp0.length();
                if r_len < 1e-6 {
                    (target.position, 0.0)
                } else {
                    let v_rel = target.velocity - us.velocity;
                    let v_c = -v_rel.dot(dp0) / r_len;
                    let t_intercept = r_len / (bullet_speed + v_c.max(0.0));
                    let intercept_pos = target.position_at(current_tick() + (t_intercept / TICK_LENGTH).round() as u32);
                    (intercept_pos, t_intercept)
                }
            }
            1 => {
                let d = (target.position - us.position).length();
                let t_intercept = d / 750.0;
                let intercept_pos = target.position_at(current_tick() + (t_intercept / TICK_LENGTH).round() as u32);
                (intercept_pos, t_intercept)
            }
            _ => (target.position, 0.0),
        };
        Some(FiringSolution {
            aim_dir,
            omega,
            intercept_pos,
            intercept_time,
        })
    }

    pub fn fire_at(&mut self, solution: &FiringSolution) {
        if (self.reload_ticks_fn)(self.gun_id) != 0 {
            return;
        }
        self.fire();

        if self.gun_id != 0 {
            return;
        }

        let h = heading();
        let heading_vec = Vec2::new(h.cos(), h.sin());
        let gun_pos = position() + Vec2::new(20.0, 0.0).rotate(h);
        let bullet_vel = velocity() + heading_vec * 1000.0;
        let actual_pos = gun_pos + bullet_vel * solution.intercept_time;

        let intercept_tick = current_tick() + (solution.intercept_time / TICK_LENGTH).round() as u32;
        self.debug_intercepts.push(DebugIntercept {
            pos: solution.intercept_pos,
            actual_pos,
            intercept_tick,
        });
    }

    pub fn draw_debug(&mut self) {
        let now = current_tick();
        self.debug_intercepts.retain(|intercept| intercept.intercept_tick >= now);
        for intercept in &self.debug_intercepts {
            // Draw expected intercept diamond
            let p_exp = intercept.pos;
            let size = 15.0;
            let p1_exp = p_exp + Vec2::new(0.0, size);
            let p2_exp = p_exp + Vec2::new(size, 0.0);
            let p3_exp = p_exp + Vec2::new(0.0, -size);
            let p4_exp = p_exp + Vec2::new(-size, 0.0);
            let color_exp = rgb(0, 240, 255);
            draw_line(p1_exp, p2_exp, color_exp);
            draw_line(p2_exp, p3_exp, color_exp);
            draw_line(p3_exp, p4_exp, color_exp);
            draw_line(p4_exp, p1_exp, color_exp);

            // Draw actual landing/intercept diamond given current heading/velocity
            let p_act = intercept.actual_pos;
            let p1_act = p_act + Vec2::new(0.0, size);
            let p2_act = p_act + Vec2::new(size, 0.0);
            let p3_act = p_act + Vec2::new(0.0, -size);
            let p4_act = p_act + Vec2::new(-size, 0.0);
            let color_act = rgb(255, 0, 128); // neon magenta/rose
            draw_line(p1_act, p2_act, color_act);
            draw_line(p2_act, p3_act, color_act);
            draw_line(p3_act, p4_act, color_act);
            draw_line(p4_act, p1_act, color_act);
        }
    }
}

pub struct GunAimer {
    pub offset: Vec2,
    pub bullet_speed: f64,
}

impl GunAimer {
    pub const fn new(offset: Vec2, bullet_speed: f64) -> Self {
        Self { offset, bullet_speed }
    }
}

impl AimAt for GunAimer {
    fn aim_at(
        &self,
        target: &KinematicState,
        us: &KinematicState,
    ) -> Option<(Vec2, f64)> {
        let us_heading = us.heading.unwrap_or(0.0);
        let gun_pos = us.position + self.offset.rotate(us_heading);
        let dp0 = target.position_at(current_tick()) - gun_pos;
        let r_len = dp0.length();
        if r_len < 1e-6 {
            return None;
        }
        let target_vel = target.velocity_at(current_tick());
        let dv = target_vel - us.velocity;
        let v_c = -dv.dot(dp0) / r_len;
        let t0 = r_len / (self.bullet_speed + v_c.max(0.0));
        
        let f = |t: f64| {
            let tick_at_t = current_tick() + (t / TICK_LENGTH).round() as u32;
            let p_e = target.position_at(tick_at_t);
            let d = p_e - gun_pos - t * us.velocity;
            d.length() - self.bullet_speed * t
        };

        let df = |t: f64| {
            let tick_at_t = current_tick() + (t / TICK_LENGTH).round() as u32;
            let p_e = target.position_at(tick_at_t);
            let d = p_e - gun_pos - t * us.velocity;
            let d_len = d.length();
            let target_vel_at_t = target.velocity_at(tick_at_t);
            let d_prime = target_vel_at_t + 0.5 * target.acceleration * TICK_LENGTH - us.velocity;
            if d_len > 1e-6 {
                d.dot(d_prime) / d_len - self.bullet_speed
            } else {
                -self.bullet_speed
            }
        };

        let clamp = |t: f64| t.max(0.0);

        if let Some(t) = crate::control::newton_solve(t0, f, df, clamp, 20, 1e-4) {
            if t >= 0.0 {
                let tick_at_impact = current_tick() + (t / TICK_LENGTH).round() as u32;
                let p_e = target.position_at(tick_at_impact);
                let d = p_e - gun_pos - t * us.velocity;
                let d_len = d.length();
                if d_len > 0.0 {
                    let aim_dir = d.normalize();

                    // To compute omega for the aim point now, extrapolate the aim point next tick.
                    let us_heading_next = us_heading + us.angular_velocity.unwrap_or(0.0) * TICK_LENGTH;
                    let us_pos_next = us.position + us.velocity * TICK_LENGTH;
                    let gun_pos_next = us_pos_next + self.offset.rotate(us_heading_next);

                    let f_next = |t_next: f64| {
                        let tick_at_t = (current_tick() + 1) + (t_next / TICK_LENGTH).round() as u32;
                        let p_e = target.position_at(tick_at_t);
                        let d_n = p_e - gun_pos_next - t_next * us.velocity;
                        d_n.length() - self.bullet_speed * t_next
                    };

                    let df_next = |t_next: f64| {
                        let tick_at_t = (current_tick() + 1) + (t_next / TICK_LENGTH).round() as u32;
                        let p_e = target.position_at(tick_at_t);
                        let d_n = p_e - gun_pos_next - t_next * us.velocity;
                        let d_len = d_n.length();
                        let target_vel_at_t = target.velocity_at(tick_at_t);
                        let d_prime = target_vel_at_t + 0.5 * target.acceleration * TICK_LENGTH - us.velocity;
                        if d_len > 1e-6 {
                            d_n.dot(d_prime) / d_len - self.bullet_speed
                        } else {
                            -self.bullet_speed
                        }
                    };

                    let mut t_next_solved = t;
                    if let Some(t_n) = crate::control::newton_solve(t, f_next, df_next, clamp, 20, 1e-4) {
                        t_next_solved = t_n;
                    }

                    let tick_at_impact_next = (current_tick() + 1) + (t_next_solved / TICK_LENGTH).round() as u32;
                    let p_e_next = target.position_at(tick_at_impact_next);
                    let d_next = p_e_next - gun_pos_next - t_next_solved * us.velocity;
                    
                    let omega = if d_next.length() > 0.0 {
                        angle_diff(aim_dir.angle(), d_next.angle()) / TICK_LENGTH
                    } else {
                        0.0
                    };

                    return Some((aim_dir, omega));
                }
            }
        }
        None
    }
}

#[cfg(test)]
mod aim_test {
    use super::*;

    #[test]
    fn test_gun_aimer() {
        let us = KinematicState {
            class: Class::Fighter,
            position: Vec2::new(0.0, 0.0),
            velocity: Vec2::new(0.0, 0.0),
            acceleration: Vec2::new(0.0, 0.0),
            last_scanned: 0,
            heading: Some(0.0),
            angular_velocity: Some(0.0),
        };
        let target = KinematicState {
            class: Class::Fighter,
            position: Vec2::new(100.0, 0.0),
            velocity: Vec2::new(0.0, 10.0),
            acceleration: Vec2::new(0.0, 0.0),
            last_scanned: 0,
            heading: None,
            angular_velocity: None,
        };
        let gun_aimer = GunAimer::new(Vec2::new(0.0, 0.0), 100.0);
        let res = gun_aimer.aim_at(&target, &us);
        assert!(res.is_some());
        let (dir, omega) = res.unwrap();
        assert!(dir.x > 0.0);
        assert!(dir.y > 0.0);
        assert!(omega > 0.0);
    }

    #[test]
    fn test_gun_aimer_offset() {
        let us = KinematicState {
            class: Class::Fighter,
            position: Vec2::new(0.0, 0.0),
            velocity: Vec2::new(0.0, 0.0),
            acceleration: Vec2::new(0.0, 0.0),
            last_scanned: 0,
            heading: Some(0.0),
            angular_velocity: Some(0.0),
        };
        let target = KinematicState {
            class: Class::Fighter,
            position: Vec2::new(100.0, 0.0),
            velocity: Vec2::new(0.0, 10.0),
            acceleration: Vec2::new(0.0, 0.0),
            last_scanned: 0,
            heading: None,
            angular_velocity: None,
        };
        let gun_aimer = GunAimer::new(Vec2 { x: 20.0, y: 0.0 }, 100.0);
        let res = gun_aimer.aim_at(&target, &us);
        assert!(res.is_some());
        let (dir, omega) = res.unwrap();
        assert!(dir.x > 0.0);
        assert!(dir.y > 0.0);
        assert!(omega > 0.0);
    }

    #[test]
    fn test_missile_aimer() {
        use crate::missile::MissileAimer;

        let us = KinematicState {
            class: Class::Fighter,
            position: Vec2::new(0.0, 0.0),
            velocity: Vec2::new(0.0, 0.0),
            acceleration: Vec2::new(0.0, 0.0),
            last_scanned: 0,
            heading: Some(0.0),
            angular_velocity: Some(0.0),
        };
        let target = KinematicState {
            class: Class::Fighter,
            position: Vec2::new(100.0, 0.0),
            velocity: Vec2::new(0.0, 10.0),
            acceleration: Vec2::new(0.0, 0.0),
            last_scanned: 0,
            heading: None,
            angular_velocity: None,
        };
        let missile_aimer = MissileAimer::new(200.0, 100.0);
        let res = missile_aimer.aim_at(&target, &us);
        assert!(res.is_some());
        let (dir, omega) = res.unwrap();
        assert!(dir.x > 0.0);
        assert!(dir.y > 0.0);
        assert!(omega > 0.0);
    }
}
