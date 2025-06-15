//! Camera system for 3D point cloud visualization
//!
//! This module provides camera functionality including perspective projection,
//! view matrix calculation, and interactive camera controls.

use std::f32::consts::PI;
use winit::event::{ElementState, KeyEvent, MouseButton, WindowEvent};
use winit::keyboard::{KeyCode, PhysicalKey};

/// 3D camera for point cloud visualization
#[derive(Debug, Clone)]
pub struct Camera {
    /// Camera position in world space
    pub position: [f32; 3],

    /// Camera target point (where the camera is looking)
    pub target: [f32; 3],

    /// Up vector for camera orientation
    pub up: [f32; 3],

    /// Field of view in radians
    pub fov: f32,

    /// Aspect ratio (width / height)
    pub aspect: f32,

    /// Near clipping plane distance
    pub near: f32,

    /// Far clipping plane distance
    pub far: f32,
}

impl Camera {
    /// Create a new camera with default parameters
    pub fn new(aspect: f32) -> Self {
        Self {
            position: [0.0, 0.0, 5.0],
            target: [0.0, 0.0, 0.0],
            up: [0.0, 1.0, 0.0],
            fov: PI / 4.0, // 45 degrees
            aspect,
            near: 0.1,
            far: 1000.0,
        }
    }

    /// Update the aspect ratio (typically called on window resize)
    pub fn set_aspect(&mut self, aspect: f32) {
        self.aspect = aspect;
    }

    /// Get the view matrix for this camera
    pub fn view_matrix(&self) -> [[f32; 4]; 4] {
        let forward = normalize(subtract(self.target, self.position));
        let right = normalize(cross(forward, self.up));
        let up = cross(right, forward);

        let translation = [
            -dot(right, self.position),
            -dot(up, self.position),
            dot(forward, self.position),
        ];

        [
            [right[0], up[0], -forward[0], 0.0],
            [right[1], up[1], -forward[1], 0.0],
            [right[2], up[2], -forward[2], 0.0],
            [translation[0], translation[1], translation[2], 1.0],
        ]
    }

    /// Get the projection matrix for this camera
    pub fn projection_matrix(&self) -> [[f32; 4]; 4] {
        let f = 1.0 / (self.fov / 2.0).tan();
        let range_inv = 1.0 / (self.near - self.far);

        [
            [f / self.aspect, 0.0, 0.0, 0.0],
            [0.0, f, 0.0, 0.0],
            [0.0, 0.0, (self.near + self.far) * range_inv, -1.0],
            [0.0, 0.0, 2.0 * self.near * self.far * range_inv, 0.0],
        ]
    }

    /// Get the combined view-projection matrix
    pub fn view_projection_matrix(&self) -> [[f32; 4]; 4] {
        multiply_matrices(self.projection_matrix(), self.view_matrix())
    }

    /// Move the camera to look at a specific point
    pub fn look_at(&mut self, position: [f32; 3], target: [f32; 3], up: [f32; 3]) {
        self.position = position;
        self.target = target;
        self.up = up;
    }

    /// Get the forward direction vector
    pub fn forward(&self) -> [f32; 3] {
        normalize(subtract(self.target, self.position))
    }

    /// Get the right direction vector
    pub fn right(&self) -> [f32; 3] {
        normalize(cross(self.forward(), self.up))
    }
}

/// Interactive camera controller for handling user input
#[derive(Debug)]
pub struct CameraController {
    /// Movement speed
    pub speed: f32,

    /// Mouse sensitivity for rotation
    pub sensitivity: f32,

    /// Zoom speed for mouse wheel
    pub zoom_speed: f32,

    /// Current key states
    keys_pressed: std::collections::HashSet<KeyCode>,

    /// Mouse button states
    mouse_pressed: std::collections::HashSet<MouseButton>,

    /// Last mouse position
    last_mouse_pos: Option<(f64, f64)>,

    /// Spherical coordinates for orbit camera
    radius: f32,
    theta: f32, // Horizontal angle
    phi: f32,   // Vertical angle
}

impl CameraController {
    /// Create a new camera controller
    pub fn new(speed: f32, sensitivity: f32, zoom_speed: f32) -> Self {
        Self {
            speed,
            sensitivity,
            zoom_speed,
            keys_pressed: std::collections::HashSet::new(),
            mouse_pressed: std::collections::HashSet::new(),
            last_mouse_pos: None,
            radius: 5.0,
            theta: 0.0,
            phi: PI / 2.0,
        }
    }

    /// Process window events and update camera state
    pub fn process_event(&mut self, event: &WindowEvent) -> bool {
        match event {
            WindowEvent::KeyboardInput {
                event:
                    KeyEvent {
                        physical_key: PhysicalKey::Code(keycode),
                        state,
                        ..
                    },
                ..
            } => {
                match state {
                    ElementState::Pressed => {
                        self.keys_pressed.insert(*keycode);
                    }
                    ElementState::Released => {
                        self.keys_pressed.remove(keycode);
                    }
                }
                true
            }
            WindowEvent::MouseInput { button, state, .. } => {
                match state {
                    ElementState::Pressed => {
                        self.mouse_pressed.insert(*button);
                    }
                    ElementState::Released => {
                        self.mouse_pressed.remove(button);
                    }
                }
                true
            }
            WindowEvent::CursorMoved { position, .. } => {
                let current_pos = (position.x, position.y);

                if let Some(last_pos) = self.last_mouse_pos {
                    if self.mouse_pressed.contains(&MouseButton::Left) {
                        let dx = (current_pos.0 - last_pos.0) as f32;
                        let dy = (current_pos.1 - last_pos.1) as f32;

                        self.theta -= dx * self.sensitivity;
                        self.phi = (self.phi - dy * self.sensitivity).clamp(0.1, PI - 0.1);
                    }
                }

                self.last_mouse_pos = Some(current_pos);
                true
            }
            WindowEvent::MouseWheel { delta, .. } => {
                match delta {
                    winit::event::MouseScrollDelta::LineDelta(_, y) => {
                        self.radius = (self.radius - y * self.zoom_speed).max(0.1);
                    }
                    winit::event::MouseScrollDelta::PixelDelta(pos) => {
                        self.radius =
                            (self.radius - pos.y as f32 * self.zoom_speed * 0.01).max(0.1);
                    }
                }
                true
            }
            _ => false,
        }
    }

    /// Update camera based on current input state
    pub fn update_camera(&self, camera: &mut Camera, dt: f32) {
        // Handle keyboard movement
        let mut movement = [0.0, 0.0, 0.0];

        if self.keys_pressed.contains(&KeyCode::KeyW) {
            movement[2] -= 1.0;
        }
        if self.keys_pressed.contains(&KeyCode::KeyS) {
            movement[2] += 1.0;
        }
        if self.keys_pressed.contains(&KeyCode::KeyA) {
            movement[0] -= 1.0;
        }
        if self.keys_pressed.contains(&KeyCode::KeyD) {
            movement[0] += 1.0;
        }
        if self.keys_pressed.contains(&KeyCode::Space) {
            movement[1] += 1.0;
        }
        if self.keys_pressed.contains(&KeyCode::ShiftLeft) {
            movement[1] -= 1.0;
        }

        // Apply movement
        if movement != [0.0, 0.0, 0.0] {
            let forward = camera.forward();
            let right = camera.right();
            let up = camera.up;

            let move_vec = [
                right[0] * movement[0] + up[0] * movement[1] + forward[0] * movement[2],
                right[1] * movement[0] + up[1] * movement[1] + forward[1] * movement[2],
                right[2] * movement[0] + up[2] * movement[1] + forward[2] * movement[2],
            ];

            let move_speed = self.speed * dt;
            camera.position[0] += move_vec[0] * move_speed;
            camera.position[1] += move_vec[1] * move_speed;
            camera.position[2] += move_vec[2] * move_speed;

            camera.target[0] += move_vec[0] * move_speed;
            camera.target[1] += move_vec[1] * move_speed;
            camera.target[2] += move_vec[2] * move_speed;
        }

        // Update orbit camera position
        let x = self.radius * self.phi.sin() * self.theta.cos();
        let y = self.radius * self.phi.cos();
        let z = self.radius * self.phi.sin() * self.theta.sin();

        if self.mouse_pressed.contains(&MouseButton::Left) {
            camera.position = [x, y, z];
        }
    }
}

// Helper functions for vector math
fn subtract(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn normalize(v: [f32; 3]) -> [f32; 3] {
    let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if len > 0.0 {
        [v[0] / len, v[1] / len, v[2] / len]
    } else {
        [0.0, 0.0, 0.0]
    }
}

fn multiply_matrices(a: [[f32; 4]; 4], b: [[f32; 4]; 4]) -> [[f32; 4]; 4] {
    let mut result = [[0.0; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            for k in 0..4 {
                result[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    result
}
