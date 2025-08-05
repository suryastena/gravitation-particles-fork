use std::simd::*;

use crate::utils::world_to_screen_coords;
use ggez::{
    graphics::{self, Canvas, Color},
    mint::Point2,
    Context,
};
use nalgebra::Vector2;

use crate::consts::{G, LANES, SOFTENING};

#[derive(Clone, Debug)]
pub struct ParticleSystem {
    // Position vectors
    pub pos_x: Vec<f32>,
    pub pos_y: Vec<f32>,

    // Velocity vectors
    pub vel_x: Vec<f32>,
    pub vel_y: Vec<f32>,

    // Net force vectors
    pub net_force_x: Vec<f32>,
    pub net_force_y: Vec<f32>,

    // Scalar properties
    pub mass: Vec<f32>,
    pub radius: Vec<f32>,
    pub indices: Vec<usize>,

    // Number of particles
    pub count: usize,
}

impl ParticleSystem {
    #[allow(dead_code)]
    pub fn new() -> Self {
        Self {
            pos_x: Vec::new(),
            pos_y: Vec::new(),
            vel_x: Vec::new(),
            vel_y: Vec::new(),
            net_force_x: Vec::new(),
            net_force_y: Vec::new(),
            mass: Vec::new(),
            radius: Vec::new(),
            indices: Vec::new(),
            count: 0,
        }
    }

    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            pos_x: Vec::with_capacity(capacity),
            pos_y: Vec::with_capacity(capacity),
            vel_x: Vec::with_capacity(capacity),
            vel_y: Vec::with_capacity(capacity),
            net_force_x: Vec::with_capacity(capacity),
            net_force_y: Vec::with_capacity(capacity),
            mass: Vec::with_capacity(capacity),
            radius: Vec::with_capacity(capacity),
            indices: Vec::with_capacity(capacity),
            count: 0,
        }
    }

    pub fn add_particle(
        &mut self,
        pos: Vector2<f32>,
        vel: Vector2<f32>,
        mass: f32,
        radius: f32,
        index: usize,
    ) {
        self.pos_x.push(pos.x);
        self.pos_y.push(pos.y);
        self.vel_x.push(vel.x);
        self.vel_y.push(vel.y);
        self.net_force_x.push(0.0);
        self.net_force_y.push(0.0);
        self.mass.push(mass);
        self.radius.push(radius);
        self.indices.push(index);
        self.count += 1;
    }

    pub fn get_position(&self, idx: usize) -> Vector2<f32> {
        Vector2::new(self.pos_x[idx], self.pos_y[idx])
    }

    pub fn set_position(&mut self, idx: usize, pos: Vector2<f32>) {
        self.pos_x[idx] = pos.x;
        self.pos_y[idx] = pos.y;
    }

    pub fn get_velocity(&self, idx: usize) -> Vector2<f32> {
        Vector2::new(self.vel_x[idx], self.vel_y[idx])
    }

    pub fn set_velocity(&mut self, idx: usize, vel: Vector2<f32>) {
        self.vel_x[idx] = vel.x;
        self.vel_y[idx] = vel.y;
    }

    pub fn get_net_force(&self, idx: usize) -> Vector2<f32> {
        Vector2::new(self.net_force_x[idx], self.net_force_y[idx])
    }

    #[allow(dead_code)]
    pub fn set_net_force(&mut self, idx: usize, force: Vector2<f32>) {
        self.net_force_x[idx] = force.x;
        self.net_force_y[idx] = force.y;
    }

    pub fn reset_net_force(&mut self, idx: usize) {
        self.net_force_x[idx] = 0.0;
        self.net_force_y[idx] = 0.0;
    }

    pub fn reset_all_net_force(&mut self) {
        for f in &mut self.net_force_x {
            *f = 0.0;
        }
        for f in &mut self.net_force_y {
            *f = 0.0;
        }
    }

    pub fn add_to_net_force(&mut self, idx: usize, force: Vector2<f32>) {
        self.net_force_x[idx] += force.x;
        self.net_force_y[idx] += force.y;
    }

    pub fn apply_forces_simd(&mut self) {
        const LANES: usize = 8;
        let mut i = 0;
        while i + LANES <= self.count {
            let mass = Simd::<f32, LANES>::from_slice(&self.mass[i..i + LANES]);
            let force_x = Simd::<f32, LANES>::from_slice(&self.net_force_x[i..i + LANES]);
            let force_y = Simd::<f32, LANES>::from_slice(&self.net_force_y[i..i + LANES]);
            let mut vel_x = Simd::<f32, LANES>::from_slice(&self.vel_x[i..i + LANES]);
            let mut vel_y = Simd::<f32, LANES>::from_slice(&self.vel_y[i..i + LANES]);
            let mut pos_x = Simd::<f32, LANES>::from_slice(&self.pos_x[i..i + LANES]);
            let mut pos_y = Simd::<f32, LANES>::from_slice(&self.pos_y[i..i + LANES]);

            let acc_x = force_x / mass;
            let acc_y = force_y / mass;

            vel_x += acc_x;
            vel_y += acc_y;
            pos_x += vel_x;
            pos_y += vel_y;

            self.vel_x[i..i + LANES].copy_from_slice(&vel_x.to_array());
            self.vel_y[i..i + LANES].copy_from_slice(&vel_y.to_array());
            self.pos_x[i..i + LANES].copy_from_slice(&pos_x.to_array());
            self.pos_y[i..i + LANES].copy_from_slice(&pos_y.to_array());

            i += LANES;
        }

        for idx in i..self.count {
            let acc_x = self.net_force_x[idx] / self.mass[idx];
            let acc_y = self.net_force_y[idx] / self.mass[idx];
            self.vel_x[idx] += acc_x;
            self.vel_y[idx] += acc_y;
            self.pos_x[idx] += self.vel_x[idx];
            self.pos_y[idx] += self.vel_y[idx];
        }
    }

    pub fn get_attraction_force(&self, idx1: usize, idx2: usize) -> Vector2<f32> {
        let dx = self.pos_x[idx2] - self.pos_x[idx1];
        let dy = self.pos_y[idx2] - self.pos_y[idx1];

        let distance_squared = dx * dx + dy * dy;
        let r = (distance_squared + SOFTENING.powi(2)).sqrt();

        let norm = (dx * dx + dy * dy).sqrt();
        let dir_x = dx / norm;
        let dir_y = dy / norm;

        let magnitude = G * ((self.mass[idx1] * self.mass[idx2]) / r.powi(2));

        Vector2::new(dir_x * magnitude, dir_y * magnitude)
    }

    pub fn get_distance_to(&self, idx: usize, object: &Vector2<f32>) -> f32 {
        f32::hypot(object.x - self.pos_x[idx], object.y - self.pos_y[idx])
    }

    pub fn get_velocity_norm(&self, idx: usize) -> f32 {
        (self.vel_x[idx] * self.vel_x[idx] + self.vel_y[idx] * self.vel_y[idx]).sqrt()
    }

    fn get_color(&self, value: f32, left: &Color, right: &Color) -> Color {
        Color::from_rgb(
            (((1.0 - value) * left.r + value * right.r) * 255.0) as u8,
            (((1.0 - value) * left.g + value * right.g) * 255.0) as u8,
            (((1.0 - value) * left.b + value * right.b) * 255.0) as u8,
        )
    }

    pub fn show_particle(
        &self,
        idx: usize,
        canvas: &mut Canvas,
        ctx: &mut Context,
        offset: Vector2<f32>,
        zoom: f32,
        max_vel: f32,
        min_vel: f32,
    ) {
        let mut new_radius: f32;
        if self.radius[idx] < 1.0 {
            new_radius = 0.25 * zoom;
        } else {
            new_radius = self.radius[idx] * zoom;
        }
        if new_radius < 0.25 {
            new_radius = 0.25;
        }

        let mid_vel = (max_vel + min_vel) / 2.0;
        let left = Color::BLUE;
        let middle = Color::GREEN;
        let right = Color::RED;
        let norm_vel = self.get_velocity_norm(idx);
        let new_color: Color;

        if norm_vel < min_vel + mid_vel {
            new_color = self.get_color((norm_vel - min_vel) / mid_vel, &left, &right);
        } else {
            new_color = self.get_color((norm_vel - min_vel - mid_vel) / mid_vel, &middle, &right);
        }

        let particle_pos = self.get_position(idx);
        let dot_mesh = graphics::Mesh::new_circle(
            ctx,
            graphics::DrawMode::fill(),
            Point2 {
                x: world_to_screen_coords(particle_pos, &offset, zoom).x,
                y: world_to_screen_coords(particle_pos, &offset, zoom).y,
            },
            new_radius,
            0.1,
            new_color,
        )
        .unwrap();

        canvas.draw(&dot_mesh, graphics::DrawParam::default());
    }

    pub fn sort_by_mass(&mut self) {
        // Create indices for sorting
        let mut indices: Vec<usize> = (0..self.count).collect();
        indices.sort_by_key(|&i| self.mass[i] as u32);

        // Create new arrays in sorted order
        let mut new_pos_x = Vec::with_capacity(self.count);
        let mut new_pos_y = Vec::with_capacity(self.count);
        let mut new_vel_x = Vec::with_capacity(self.count);
        let mut new_vel_y = Vec::with_capacity(self.count);
        let mut new_net_force_x = Vec::with_capacity(self.count);
        let mut new_net_force_y = Vec::with_capacity(self.count);
        let mut new_mass = Vec::with_capacity(self.count);
        let mut new_radius = Vec::with_capacity(self.count);
        let mut new_indices = Vec::with_capacity(self.count);

        for &i in &indices {
            new_pos_x.push(self.pos_x[i]);
            new_pos_y.push(self.pos_y[i]);
            new_vel_x.push(self.vel_x[i]);
            new_vel_y.push(self.vel_y[i]);
            new_net_force_x.push(self.net_force_x[i]);
            new_net_force_y.push(self.net_force_y[i]);
            new_mass.push(self.mass[i]);
            new_radius.push(self.radius[i]);
            new_indices.push(self.indices[i]);
        }

        self.pos_x = new_pos_x;
        self.pos_y = new_pos_y;
        self.vel_x = new_vel_x;
        self.vel_y = new_vel_y;
        self.net_force_x = new_net_force_x;
        self.net_force_y = new_net_force_y;
        self.mass = new_mass;
        self.radius = new_radius;
        self.indices = new_indices;
    }

    pub fn find_max_velocity_norm(&self) -> f32 {
        use std::simd::{num::SimdFloat, Simd};

        let mut max_chunk = Simd::<f32, LANES>::splat(0.0);
        let mut i = 0;
        while i + LANES <= self.count {
            let vx = Simd::<f32, LANES>::from_slice(&self.net_force_x[i..i + LANES]);
            let vy = Simd::<f32, LANES>::from_slice(&self.net_force_y[i..i + LANES]);
            let norm = (vx * vx + vy * vy).sqrt();
            max_chunk = max_chunk.simd_max(norm);
            i += LANES;
        }
        let mut max_vel = max_chunk.reduce_max();
        for j in i..self.count {
            let norm = self.get_velocity_norm(j);
            if norm > max_vel {
                max_vel = norm;
            }
        }
        max_vel
    }

    pub fn find_min_velocity_norm(&self) -> f32 {
        use std::simd::{num::SimdFloat, Simd};

        if self.count == 0 {
            return 0.0;
        }
        const LANES: usize = 8;
        let mut min_chunk = Simd::<f32, LANES>::splat(f32::INFINITY);
        let mut i = 0;
        while i + LANES <= self.count {
            let vx = Simd::<f32, LANES>::from_slice(&self.net_force_x[i..i + LANES]);
            let vy = Simd::<f32, LANES>::from_slice(&self.net_force_y[i..i + LANES]);
            let norm = (vx * vx + vy * vy).sqrt();
            min_chunk = min_chunk.simd_min(norm);
            i += LANES;
        }
        let mut min_vel = min_chunk.reduce_min();
        for j in i..self.count {
            let norm = self.get_velocity_norm(j);
            if norm < min_vel {
                min_vel = norm;
            }
        }
        min_vel
    }
}
