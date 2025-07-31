use crate::consts::{G, MAX_ZOOM, MOUSE_AREA, WORLD_HEIGHT, WORLD_WIDTH};
use crate::particle::ParticleSystem;
use crate::quadtree::QuadTree;
use crate::rectangle::Rectangle;
use chrono::{DateTime, Local};
use ggez::graphics::{ImageEncodingFormat, ScreenImage};
use ggez::Context;
use nalgebra::Vector2;
use rand::Rng;
use std::fs;
use std::io::{BufRead, BufReader};
use std::path::PathBuf;
use std::process::{Command, Stdio};

fn random_in_circle(radius: f32, padding: f32, center: Vector2<f32>) -> Vector2<f32> {
    let mut rng = rand::thread_rng();
    let angle = rng.gen_range(0.0..2.0 * std::f32::consts::PI);
    let distance = rng.gen_range(padding..radius);

    Vector2::new(distance * angle.cos(), distance * angle.sin()) + center
}

#[allow(dead_code)]
pub fn spawn_circle(
    particles: &mut ParticleSystem,
    center: Vector2<f32>,
    radius: f32,
    particle_mass: f32,
    particles_amount: i32,
) {
    for i in 0..particles_amount {
        let pos = random_in_circle(radius, 0.0, center);
        particles.add_particle(pos, Vector2::default(), particle_mass, 0.00001, i as usize);
    }
}

#[allow(dead_code)]
pub fn create_galaxy(
    particles: &mut ParticleSystem,
    center: Vector2<f32>,
    initial_vel: Vector2<f32>,
    radius: f32,
    sun_mass: f32,
    particle_mass: f32,
    particles_amount: i32,
) {
    for i in 0..particles_amount {
        let pos = random_in_circle(radius, 2.0, center);
        let distance_to_center = pos.metric_distance(&center);
        let orbital_vel = ((G * sun_mass) / distance_to_center).sqrt();
        let dir = Vector2::new(pos.y - center.y, center.x - pos.x).normalize();
        particles.add_particle(pos, dir * orbital_vel, particle_mass, 0.00001, i as usize);
    }

    // Add the sun
    particles.add_particle(
        center,
        initial_vel,
        sun_mass,
        1.5,
        particles_amount as usize,
    );
}

pub fn create_quadtree(particles: &ParticleSystem) -> QuadTree {
    let mut qt = QuadTree::new(Rectangle::new(
        Vector2::new(0.0, 0.0),
        WORLD_WIDTH,
        WORLD_HEIGHT,
    ));
    for i in 0..particles.count {
        qt.insert(particles, i);
    }
    qt
}

pub fn world_to_screen_coords(
    world_coords: Vector2<f32>,
    origin: &Vector2<f32>,
    zoom: f32,
) -> Vector2<f32> {
    (origin + world_coords) * zoom
}

pub fn screen_to_world_coords(
    screen_coords: Vector2<f32>,
    origin: &Vector2<f32>,
    zoom: f32,
) -> Vector2<f32> {
    screen_coords / zoom - origin
}

pub fn move_on_mouse(ctx: &mut Context, origin: &mut Vector2<f32>, zoom: f32) {
    let mouse_pos = ctx.mouse.position();
    let (mouse_x, mouse_y) = (mouse_pos.x, mouse_pos.y);
    if ctx
        .mouse
        .button_pressed(ggez::input::mouse::MouseButton::Left)
    {
        let mouse_del = ctx.mouse.delta();
        let (delta_x, delta_y) = (mouse_del.x, mouse_del.y);
        origin.x += delta_x / zoom;
        origin.y += delta_y / zoom;
    }

    if ctx
        .mouse
        .button_pressed(ggez::input::mouse::MouseButton::Middle)
    {
        let screen_coords = Vector2::new(mouse_x, mouse_y);
        let world_coords = screen_to_world_coords(screen_coords, origin, zoom);
        if world_coords.x < 0.0
            || world_coords.x > WORLD_WIDTH
            || world_coords.y < 0.0
            || world_coords.y > WORLD_HEIGHT
        {
            return;
        }
    }
}

pub fn zoom_world(
    ctx: &mut Context,
    origin: &mut Vector2<f32>,
    zoom: &mut f32,
    wheel_direction: f32,
) {
    let scale_factor = 1.1;
    let mouse_pos = ctx.mouse.position();
    let (mouse_x, mouse_y) = (mouse_pos.x, mouse_pos.y);
    let mouse_world_before = screen_to_world_coords(Vector2::new(mouse_x, mouse_y), origin, *zoom);

    if wheel_direction > 0.0 {
        *zoom = (*zoom * scale_factor).min(MAX_ZOOM);
    } else if wheel_direction < 0.0 {
        *zoom = (*zoom / scale_factor).max(MOUSE_AREA);
    }

    let mouse_world_after = screen_to_world_coords(Vector2::new(mouse_x, mouse_y), origin, *zoom);
    origin.x += mouse_world_before.x - mouse_world_after.x;
    origin.y += mouse_world_before.y - mouse_world_after.y;

    origin.x = origin.x.clamp(-WORLD_WIDTH / 2.0, WORLD_WIDTH / 2.0);
    origin.y = origin.y.clamp(-WORLD_HEIGHT / 2.0, WORLD_HEIGHT / 2.0);
}

pub fn save_screen(ctx: &mut Context, screen: &mut ScreenImage, frame_count: u32) {
    let path = format!("/image-cache/frame-{}.jpg", frame_count);
    let result = screen
        .image(ctx)
        .encode(ctx, ImageEncodingFormat::Jpeg, path.as_str());
    match result {
        Ok(_) => {}
        Err(e) => eprintln!("Error saving screen: {:?}", e),
    }
}

pub fn rename_images(ctx: &Context) {
    let cache_dir_path: PathBuf = ctx.fs.resources_dir().join("image-cache");
    if !cache_dir_path.exists() || !cache_dir_path.is_dir() {
        eprintln!("Cache directory does not exist or is not a directory.");
        return;
    }

    let mut cache_pics: Vec<PathBuf> = fs::read_dir(&cache_dir_path)
        .expect("Failed to read cache directory")
        .filter_map(|entry| {
            let entry = entry.expect("Failed to get directory entry");
            let path = entry.path();
            if path.is_file() && path.extension() == Some(std::ffi::OsStr::new("jpg")) {
                Some(path)
            } else {
                None
            }
        })
        .collect();

    cache_pics.sort_by(|a, b| {
        let a_number = a
            .file_stem()
            .and_then(|s| s.to_str())
            .and_then(|s| s.trim_start_matches("frame-").parse::<u32>().ok())
            .unwrap_or(0);
        let b_number = b
            .file_stem()
            .and_then(|s| s.to_str())
            .and_then(|s| s.trim_start_matches("frame-").parse::<u32>().ok())
            .unwrap_or(0);
        a_number.cmp(&b_number)
    });

    for (index, old_path) in cache_pics.iter().enumerate() {
        let new_name = format!("{:06}.jpg", index + 1);
        let new_path = cache_dir_path.join(new_name);
        if let Err(e) = fs::rename(&old_path, &new_path) {
            eprintln!("Error renaming file: {:?}", e);
        }
    }
}

pub fn convert_to_video(ctx: &Context) {
    let now: DateTime<Local> = Local::now();
    let timestamp = now.format("%Y%m%d_%H%M%S").to_string();
    let output_filename = format!("output_{}.mp4", timestamp);
    let current_dir = std::env::current_dir().expect("Failed to get current directory");
    let results_path = current_dir.join("results").join(output_filename);

    let cache_dir_path: PathBuf = ctx.fs.resources_dir().join("image-cache");
    let input_pattern = cache_dir_path.join("%06d.jpg");

    let mut cmd = Command::new("ffmpeg")
        .args(&["-y"])
        .args(&["-framerate", "60"])
        .args(&["-i", input_pattern.to_str().expect("Invalid path")])
        .args(&["-c:v", "libx264"])
        .args(&["-pix_fmt", "yuv420p"])
        .args(&["-preset", "veryfast"])
        .args(&["-crf", "18"])
        .arg(results_path.to_str().expect("Invalid path"))
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("Failed to spawn ffmpeg");

    let stderr = cmd.stderr.take().expect("Failed to capture stderr");
    let reader = BufReader::new(stderr);
    for line in reader.lines() {
        if let Ok(line) = line {
            println!("{}", line);
        }
    }

    match cmd.wait() {
        Ok(status) => {
            if status.success() {
                println!("Video created successfully!");
            } else {
                eprintln!("ffmpeg failed with status: {}", status);
            }
        }
        Err(e) => eprintln!("Error running ffmpeg: {:?}", e),
    }
}

pub fn clean_cache_images(ctx: &Context) {
    let cache_dir_path: PathBuf = ctx.fs.resources_dir().join("image-cache");
    if !cache_dir_path.exists() || !cache_dir_path.is_dir() {
        return;
    }

    let entries = fs::read_dir(&cache_dir_path).expect("Failed to read cache directory");
    for entry in entries {
        if let Ok(entry) = entry {
            let path = entry.path();
            if path.is_file() && path.extension() == Some(std::ffi::OsStr::new("jpg")) {
                if let Err(e) = fs::remove_file(&path) {
                    eprintln!("Error deleting file: {:?}", e);
                }
            }
        }
    }
}
