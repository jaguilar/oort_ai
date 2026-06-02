use oort_api::prelude::*;
use crate::physics::KinematicState;

pub trait AimAt {
    fn aim_at(
        &self,
        target: &KinematicState,
        us: &KinematicState,
    ) -> Option<(Vec2, f64)>;
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
        let missile_aimer = MissileAimer::new(200.0);
        let res = missile_aimer.aim_at(&target, &us);
        assert!(res.is_some());
        let (dir, omega) = res.unwrap();
        assert!(dir.x > 0.0);
        assert!(dir.y > 0.0);
        assert!(omega > 0.0);
    }
}
