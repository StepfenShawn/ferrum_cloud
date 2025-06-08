//! Point types and traits
//!
//! This module defines the fundamental point types and the Point trait that
//! all point types must implement.

use serde::{Deserialize, Serialize};
use std::fmt::Debug;

/// Core trait that all point types must implement
///
/// This trait provides the basic interface for accessing point coordinates
/// and can be extended with additional methods for specific point types.
pub trait Point: Send + Sync + Clone + Debug {
    /// Get the 3D position of the point as [x, y, z]
    fn position(&self) -> [f32; 3];

    /// Get the x coordinate
    fn x(&self) -> f32 {
        self.position()[0]
    }

    /// Get the y coordinate
    fn y(&self) -> f32 {
        self.position()[1]
    }

    /// Get the z coordinate
    fn z(&self) -> f32 {
        self.position()[2]
    }

    /// Calculate squared distance to another point
    fn distance_squared_to<P: Point>(&self, other: &P) -> f32 {
        let pos1 = self.position();
        let pos2 = other.position();
        let dx = pos1[0] - pos2[0];
        let dy = pos1[1] - pos2[1];
        let dz = pos1[2] - pos2[2];
        dx * dx + dy * dy + dz * dz
    }

    /// Calculate distance to another point
    fn distance_to<P: Point>(&self, other: &P) -> f32 {
        self.distance_squared_to(other).sqrt()
    }
}

/// Basic 3D point with XYZ coordinates
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PointXYZ {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl PointXYZ {
    /// Create a new PointXYZ
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    /// Create a point at the origin
    pub fn origin() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }

    /// Create from array
    pub fn from_array(coords: [f32; 3]) -> Self {
        Self::new(coords[0], coords[1], coords[2])
    }
}

impl Point for PointXYZ {
    fn position(&self) -> [f32; 3] {
        [self.x, self.y, self.z]
    }
}

impl Default for PointXYZ {
    fn default() -> Self {
        Self::origin()
    }
}

/// 3D point with XYZ coordinates and RGB color
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PointXYZRGB {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

impl PointXYZRGB {
    /// Create a new PointXYZRGB
    pub fn new(x: f32, y: f32, z: f32, r: u8, g: u8, b: u8) -> Self {
        Self { x, y, z, r, g, b }
    }

    /// Create from coordinates and packed RGB
    pub fn from_coords_and_rgb(x: f32, y: f32, z: f32, rgb: u32) -> Self {
        let r = ((rgb >> 16) & 0xFF) as u8;
        let g = ((rgb >> 8) & 0xFF) as u8;
        let b = (rgb & 0xFF) as u8;
        Self::new(x, y, z, r, g, b)
    }

    /// Get RGB as packed u32
    pub fn rgb(&self) -> u32 {
        ((self.r as u32) << 16) | ((self.g as u32) << 8) | (self.b as u32)
    }

    /// Get RGB as normalized floats [0.0, 1.0]
    pub fn rgb_normalized(&self) -> [f32; 3] {
        [
            self.r as f32 / 255.0,
            self.g as f32 / 255.0,
            self.b as f32 / 255.0,
        ]
    }
}

impl Point for PointXYZRGB {
    fn position(&self) -> [f32; 3] {
        [self.x, self.y, self.z]
    }
}

impl Default for PointXYZRGB {
    fn default() -> Self {
        Self::new(0.0, 0.0, 0.0, 0, 0, 0)
    }
}

/// 3D point with XYZ coordinates, RGB color, and normal vector
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PointXYZRGBNormal {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub r: u8,
    pub g: u8,
    pub b: u8,
    pub normal_x: f32,
    pub normal_y: f32,
    pub normal_z: f32,
}

impl PointXYZRGBNormal {
    /// Create a new PointXYZRGBNormal
    pub fn new(
        x: f32,
        y: f32,
        z: f32,
        r: u8,
        g: u8,
        b: u8,
        normal_x: f32,
        normal_y: f32,
        normal_z: f32,
    ) -> Self {
        Self {
            x,
            y,
            z,
            r,
            g,
            b,
            normal_x,
            normal_y,
            normal_z,
        }
    }

    /// Get the normal vector
    pub fn normal(&self) -> [f32; 3] {
        [self.normal_x, self.normal_y, self.normal_z]
    }

    /// Get RGB as packed u32
    pub fn rgb(&self) -> u32 {
        ((self.r as u32) << 16) | ((self.g as u32) << 8) | (self.b as u32)
    }
}

impl Point for PointXYZRGBNormal {
    fn position(&self) -> [f32; 3] {
        [self.x, self.y, self.z]
    }
}

impl Default for PointXYZRGBNormal {
    fn default() -> Self {
        Self::new(0.0, 0.0, 0.0, 0, 0, 0, 0.0, 0.0, 1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_point_xyz_creation() {
        let point = PointXYZ::new(1.0, 2.0, 3.0);
        assert_eq!(point.position(), [1.0, 2.0, 3.0]);
        assert_eq!(point.x(), 1.0);
        assert_eq!(point.y(), 2.0);
        assert_eq!(point.z(), 3.0);
    }

    #[test]
    fn test_point_distance() {
        let p1 = PointXYZ::new(0.0, 0.0, 0.0);
        let p2 = PointXYZ::new(3.0, 4.0, 0.0);
        assert_eq!(p1.distance_to(&p2), 5.0);
        assert_eq!(p1.distance_squared_to(&p2), 25.0);
    }

    #[test]
    fn test_point_xyz_rgb() {
        let point = PointXYZRGB::new(1.0, 2.0, 3.0, 255, 128, 64);
        assert_eq!(point.position(), [1.0, 2.0, 3.0]);
        assert_eq!(point.rgb(), 0xFF8040);

        let normalized = point.rgb_normalized();
        assert!((normalized[0] - 1.0).abs() < f32::EPSILON);
        assert!((normalized[1] - 0.5019608).abs() < 0.001);
    }
}
