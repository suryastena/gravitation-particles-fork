#![feature(portable_simd)]

mod consts;
mod particle;
mod quadtree;
mod rectangle;
mod utils;

use consts::{HEIGHT, MAX_ZOOM, WIDTH, WORLD_HEIGHT, WORLD_WIDTH};
use ggez::event::{self, EventHandler};
use ggez::graphics::{self, Color};
use ggez::input::keyboard::{KeyCode, KeyInput};
use ggez::GameError;
use ggez::{conf, Context, ContextBuilder, GameResult};
use nalgebra::Vector2;
use particle::ParticleSystem;
use quadtree::QuadTree;
use rectangle::Rectangle;
use std::sync::{Arc, Mutex};
use std::{env, fs};
use utils::{
    clean_cache_images, convert_to_video, create_galaxy, create_quadtree, create_square,
    move_on_mouse, rename_images, save_screen, screen_to_world_coords, spawn_circle, zoom_world,
};

fn main() {
    let window_setup = conf::WindowSetup::default().title("Gravity Particles");
    let window_mode = conf::WindowMode::default()
        .dimensions(WIDTH, HEIGHT)
        .fullscreen_type(conf::FullscreenType::Windowed)
        .resizable(true);
    let (mut ctx, event_loop) = ContextBuilder::new("gravity", "xanin")
        .window_setup(window_setup)
        .window_mode(window_mode)
        .build()
        .expect("aieee, could not create ggez context!");

    match ctx.fs.create_dir("image-cache") {
        Ok(_) => println!("Created initial cache folder"),
        Err(creating_error) => eprintln!("Error creating folder: {:?}", creating_error),
    }
    let directory_name = "results";
    let current_dir = env::current_dir().expect("Failed to get current directory");
    let new_directory_path = current_dir.join(directory_name);
    match fs::metadata(&new_directory_path) {
        Ok(_) => println!("Results folder already exists"),
        Err(_) => match fs::create_dir(&new_directory_path) {
            Ok(_) => {
                println!("Created initial results folder");
            }
            Err(e) => {
                eprintln!("Error creating directory: {}", e);
            }
        },
    }

    let my_game = MyGame::new(&mut ctx);

    event::run(ctx, event_loop, my_game);
}

struct MyGame {
    screen: graphics::ScreenImage,
    qt: Arc<Mutex<QuadTree>>,
    particles: ParticleSystem,
    force_idxs: Vec<usize>,
    keysdown: Vec<KeyCode>,
    origin: Vector2<f32>,
    zoom: f32,
    frame_count: u32,
    recording: bool,
    max_vel_avg: f32,
    min_vel_avg: f32,
    vel_amount: u32,
    // new fields for optimization of drawing/title and velocity sampling
    last_fps: u32,
    last_recording: bool,
    vel_sample_counter: u32,
    sample_interval: u32,
    cached_max_vel: f32,
    cached_min_vel: f32,
}

impl MyGame {
    pub fn new(ctx: &mut Context) -> MyGame {
        let origin = Vector2::new(-100.0, -100.0);
        let zoom = MAX_ZOOM;
        let screen =
            graphics::ScreenImage::new(ctx, graphics::ImageFormat::Rgba8UnormSrgb, 1., 1., 1);
        let qt = Arc::new(Mutex::new(QuadTree::new(Rectangle::new(
            Vector2::new(0.0, 0.0),
            WORLD_WIDTH,
            WORLD_HEIGHT,
        ))));

        let mut particles = ParticleSystem::with_capacity(4000);

        create_galaxy(
            &mut particles,
            screen_to_world_coords(Vector2::new(WIDTH / 2.0, HEIGHT / 2.0), &origin, zoom),
            Vector2::new(0.01, 0.01),
            100.0,
            10.0,
            0.01,
            4000,
        );

        let o2 = Vector2::new(-200.0, -200.0);
//        create_galaxy(
//            &mut particles,
//            screen_to_world_coords(Vector2::new(WIDTH / 2.0, HEIGHT / 2.0), &o2, zoom),
//            Vector2::new(-0.1, -0.1),
//            50.0,
//            10.0,
//            0.001,
//            500,
//        );

//        create_square(
//            &mut particles,
//            Vector2::new(100.0, 100.0),
//            200.0,
//            0.2,
//            Vector2::new(0.0, 0.0),
//            0.1,
//            100,
//        );

        // Sort particles by mass
        particles.sort_by_mass();

        MyGame {
            screen,
            qt,
            particles,
            force_idxs: Vec::with_capacity(4000),
            keysdown: Vec::new(),
            origin,
            zoom,
            frame_count: 0,
            recording: false,
            max_vel_avg: 0.0,
            min_vel_avg: 0.0,
            vel_amount: 0,
            last_fps: 0,
            last_recording: false,
            vel_sample_counter: 0,
            sample_interval: 1, // sample velocities every 5 frames
            cached_max_vel: 0.0,
            cached_min_vel: 0.0,
        }
    }
}

impl EventHandler for MyGame {
    fn update(&mut self, ctx: &mut Context) -> GameResult {
        {
            let mut qt_lock = self.qt.lock().unwrap();
            // Rebuild the quadtree in-place instead of replacing the Arc/Mutex each frame.
            *qt_lock = create_quadtree(&self.particles);
            self.particles.reset_all_net_force();
            // Reuse the same index buffer to avoid allocating every frame.
            self.force_idxs.resize(self.particles.count, 0);
            for (i, slot) in self.force_idxs.iter_mut().enumerate() {
                *slot = i;
            }
            qt_lock.calculate_force_simd(&mut self.particles, &self.force_idxs);
        }
        self.particles.apply_forces_simd();
        move_on_mouse(ctx, &mut self.origin, self.zoom);
        Ok(())
    }

    fn draw(&mut self, ctx: &mut Context) -> GameResult {
        let bg_color = Color::BLACK;
        let mut canvas = graphics::Canvas::from_screen_image(ctx, &mut self.screen, bg_color);
        let draw_query_area = Rectangle::new(
            screen_to_world_coords(Vector2::new(0.0, 0.0), &self.origin, self.zoom),
            WIDTH / self.zoom,
            HEIGHT / self.zoom,
        );

        // Sample and update velocity averages only every `sample_interval` frames.
        let fps_u32 = ctx.time.fps() as u32;
        if self.vel_sample_counter == 0 {
            let max_vel = self.particles.find_max_velocity_norm();
            let min_vel = self.particles.find_min_velocity_norm();
            self.cached_max_vel = max_vel;
            self.cached_min_vel = min_vel;

            self.max_vel_avg = (self.max_vel_avg * self.vel_amount as f32 + max_vel)
                / (self.vel_amount as f32 + 1.0);
            self.min_vel_avg = (self.min_vel_avg * self.vel_amount as f32 + min_vel)
                / (self.vel_amount as f32 + 1.0);
            self.vel_amount += 1;
        }
        self.vel_sample_counter = (self.vel_sample_counter + 1) % self.sample_interval;

        let locked_qt = self.qt.lock().unwrap();
        let particles_to_draw = locked_qt.query(&draw_query_area, &self.particles);
        locked_qt.show(
            &mut canvas,
            ctx,
            self.origin,
            self.zoom,
            &particles_to_draw,
            &self.particles,
            self.max_vel_avg,
            self.min_vel_avg,
            false,
        );

        // Update title only when fps or recording state changes.
        if self.recording {
            self.frame_count += 1;
            save_screen(ctx, &mut self.screen, self.frame_count);
        }
        if fps_u32 != self.last_fps || self.recording != self.last_recording {
            let title = if self.recording {
                format!("FPS: {} Recording...", fps_u32)
            } else {
                format!("FPS: {}", fps_u32)
            };
            ctx.gfx.set_window_title(title.as_str());
            self.last_fps = fps_u32;
            self.last_recording = self.recording;
        }
        canvas.finish(ctx)?;
        ctx.gfx.present(&self.screen.image(ctx))?;
        Ok(())
    }

    fn key_down_event(
        &mut self,
        ctx: &mut Context,
        keyinput: KeyInput,
        _repeat: bool,
    ) -> Result<(), GameError> {
        if let Some(keycode) = keyinput.keycode {
            self.keysdown.push(keycode);
            self.keysdown.dedup_by_key(|x| *x);

            if keycode == KeyCode::R {
                self.recording = true;
                println!("Recording!");
            }
            if keycode == KeyCode::S {
                self.recording = false;
                println!("Saving video to project folder (results)...");
                rename_images(ctx);
                convert_to_video(ctx);
                clean_cache_images(ctx);
                println!("Saved!");
            }
        }
        Ok(())
    }

    fn key_up_event(&mut self, _ctx: &mut Context, keyinput: KeyInput) -> Result<(), GameError> {
        if let Some(keycode) = keyinput.keycode {
            self.keysdown.retain(|&x| x != keycode);
        }
        Ok(())
    }

    fn mouse_wheel_event(&mut self, ctx: &mut Context, _x: f32, y: f32) -> Result<(), GameError> {
        zoom_world(ctx, &mut self.origin, &mut self.zoom, y);

        Ok(())
    }
}
