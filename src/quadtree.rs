use crate::particle::ParticleSystem;
use crate::rectangle::Rectangle;
use ggez::{
    graphics::{Canvas, Color},
    Context,
};
use nalgebra::Vector2;

#[derive(Clone)]
pub struct QuadTree {
    bounds: Rectangle,
    // children slots; None when leaf
    children: [Option<Box<QuadTree>>; 4],
    // stored particle index at leaf
    particle_idx: Option<usize>,
    mass: f32,
    center_of_mass: Vector2<f32>,
}

impl QuadTree {
    pub fn new(bounds: Rectangle) -> Self {
        let center = Vector2::new(
            bounds.top_left_pos.x + bounds.w * 0.5,
            bounds.top_left_pos.y + bounds.h * 0.5,
        );
        Self {
            bounds,
            children: [None, None, None, None],
            particle_idx: None,
            mass: 0.0,
            center_of_mass: center,
        }
    }

    #[inline]
    fn is_leaf(&self) -> bool {
        self.children[0].is_none()
    }

    fn subdivide(&mut self) {
        let Rectangle { top_left_pos, w, h } = self.bounds;
        let half_w = w * 0.5;
        let half_h = h * 0.5;

        let quads = [
            Rectangle::new(top_left_pos, half_w, half_h),
            Rectangle::new(
                Vector2::new(top_left_pos.x + half_w, top_left_pos.y),
                half_w,
                half_h,
            ),
            Rectangle::new(
                Vector2::new(top_left_pos.x, top_left_pos.y + half_h),
                half_w,
                half_h,
            ),
            Rectangle::new(
                Vector2::new(top_left_pos.x + half_w, top_left_pos.y + half_h),
                half_w,
                half_h,
            ),
        ];

        for (i, rect) in quads.into_iter().enumerate() {
            self.children[i] = Some(Box::new(QuadTree::new(rect)));
        }
    }

    pub fn insert(&mut self, particles: &ParticleSystem, idx: usize) {
        let pos = particles.get_position(idx);
        if !self.bounds.contains_point(&pos) {
            return;
        }

        // empty leaf: store directly
        if self.is_leaf() && self.particle_idx.is_none() {
            self.particle_idx = Some(idx);
            self.mass = particles.mass[idx];
            self.center_of_mass = pos;
            return;
        }

        // if leaf with existing particle, subdivide and re-insert
        if self.is_leaf() {
            let old_idx = self.particle_idx.take().unwrap();
            self.subdivide();
            // insert old stored particle
            self.insert(particles, old_idx);
        }

        // insert new into appropriate child
        for child_opt in &mut self.children {
            if let Some(child) = child_opt {
                if child.bounds.contains_point(&pos) {
                    child.insert(particles, idx);
                    break;
                }
            }
        }

        // incremental center-of-mass update
        let new_mass = self.mass + particles.mass[idx];
        if new_mass > 0.0 {
            self.center_of_mass =
                (self.center_of_mass * self.mass + pos * particles.mass[idx]) / new_mass;
            self.mass = new_mass;
        }
    }

    pub fn calculate_force(&self, particles: &mut ParticleSystem, idx: usize) {
        let pos = particles.get_position(idx);

        if self.is_leaf() {
            if let Some(other_idx) = self.particle_idx {
                if other_idx != idx {
                    let f = particles.get_attraction_force(idx, other_idx);
                    particles.add_to_net_force(idx, f);
                }
            }
            return;
        }

        let dx = self.center_of_mass.x - pos.x;
        let dy = self.center_of_mass.y - pos.y;
        let dist = (dx * dx + dy * dy + crate::consts::SOFTENING.powi(2)).sqrt();

        if self.bounds.w / dist < 0.5 {
            let inv = 1.0 / dist;
            let dir = Vector2::new(dx * inv, dy * inv);
            let magnitude = crate::consts::G * particles.mass[idx] * self.mass / (dist * dist);
            let force = dir * magnitude;
            particles.add_to_net_force(idx, force);
        } else {
            for child_opt in &self.children {
                if let Some(child) = child_opt {
                    child.calculate_force(particles, idx);
                }
            }
        }
    }

    pub fn query(&self, area: &Rectangle, particles: &ParticleSystem) -> Vec<usize> {
        let mut result = Vec::new();
        self.query_recursive(area, particles, &mut result);
        result
    }

    fn query_recursive(&self, area: &Rectangle, particles: &ParticleSystem, out: &mut Vec<usize>) {
        if !self.bounds.intersects(area) {
            return;
        }
        if let Some(idx) = self.particle_idx {
            if area.contains_point(&particles.get_position(idx)) {
                out.push(idx);
            }
        }
        if !self.is_leaf() {
            for child_opt in &self.children {
                if let Some(child) = child_opt {
                    child.query_recursive(area, particles, out);
                }
            }
        }
    }

    pub fn show(
        &self,
        canvas: &mut Canvas,
        ctx: &mut Context,
        offset: Vector2<f32>,
        zoom: f32,
        particle_list: &[usize],
        particles: &ParticleSystem,
        max_vel: f32,
        min_vel: f32,
        draw_bounds: bool,
    ) {
        if draw_bounds {
            self.bounds.show(canvas, ctx, offset, zoom, Color::MAGENTA);
        }
        for &i in particle_list {
            particles.show_particle(i, canvas, ctx, offset, zoom, max_vel, min_vel);
        }
        if draw_bounds && !self.is_leaf() {
            for child_opt in &self.children {
                if let Some(child) = child_opt {
                    child.show(
                        canvas,
                        ctx,
                        offset,
                        zoom,
                        particle_list,
                        particles,
                        max_vel,
                        min_vel,
                        draw_bounds,
                    );
                }
            }
        }
    }
}
