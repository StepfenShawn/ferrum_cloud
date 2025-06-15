//! Configuration types for visualization
//!
//! This module contains configuration structures for customizing the appearance
//! and behavior of the point cloud viewer.

use serde::{Deserialize, Serialize};

/// Configuration for point cloud rendering
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RenderConfig {
    /// Point size in pixels
    pub point_size: f32,

    /// Background color as RGBA
    pub background_color: [f32; 4],

    /// Default point color for points without color information
    pub default_point_color: [f32; 3],

    /// Whether to enable depth testing
    pub depth_test: bool,

    /// Whether to enable point size attenuation with distance
    pub size_attenuation: bool,

    /// Field of view in degrees
    pub fov: f32,

    /// Near clipping plane distance
    pub near_plane: f32,

    /// Far clipping plane distance
    pub far_plane: f32,
}

impl Default for RenderConfig {
    fn default() -> Self {
        Self {
            point_size: 2.0,
            background_color: [0.1, 0.1, 0.1, 1.0], // Dark gray
            default_point_color: [1.0, 1.0, 1.0],   // White
            depth_test: true,
            size_attenuation: true,
            fov: 45.0,
            near_plane: 0.1,
            far_plane: 1000.0,
        }
    }
}

/// Configuration for the point cloud viewer
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ViewerConfig {
    /// Window title
    pub title: String,

    /// Initial window width
    pub width: u32,

    /// Initial window height
    pub height: u32,

    /// Whether the window is resizable
    pub resizable: bool,

    /// Whether to enable VSync
    pub vsync: bool,

    /// Maximum frames per second (0 for unlimited)
    pub max_fps: u32,

    /// Render configuration
    pub render: RenderConfig,

    /// Camera movement speed
    pub camera_speed: f32,

    /// Mouse sensitivity for camera rotation
    pub mouse_sensitivity: f32,

    /// Zoom speed for mouse wheel
    pub zoom_speed: f32,
}

impl Default for ViewerConfig {
    fn default() -> Self {
        Self {
            title: "FerrumCloud Viewer".to_string(),
            width: 1024,
            height: 768,
            resizable: true,
            vsync: true,
            max_fps: 60,
            render: RenderConfig::default(),
            camera_speed: 5.0,
            mouse_sensitivity: 0.002,
            zoom_speed: 0.1,
        }
    }
}

/// Rendering mode for point clouds
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum RenderMode {
    /// Render points as simple dots
    Points,

    /// Render points as spheres (higher quality but slower)
    Spheres,

    /// Render points with intensity-based coloring
    Intensity,

    /// Render normal vectors as lines
    Normals,
}

impl Default for RenderMode {
    fn default() -> Self {
        Self::Points
    }
}

/// Color scheme for point cloud visualization
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum ColorScheme {
    /// Use original point colors if available
    Original,

    /// Color by height (Z coordinate)
    Height,

    /// Color by distance from origin
    Distance,

    /// Color by normal direction
    Normal,

    /// Single color for all points
    Uniform([f32; 3]),
}

impl Default for ColorScheme {
    fn default() -> Self {
        Self::Original
    }
}
