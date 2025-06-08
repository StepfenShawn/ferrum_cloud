//! Metadata for point clouds
//!
//! This module defines metadata structures that store information about
//! point cloud properties such as dimensions, organization, and sensor data.

use serde::{Deserialize, Serialize};

/// Metadata associated with a point cloud
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Metadata {
    /// Width of the point cloud (for organized clouds)
    pub width: u32,

    /// Height of the point cloud (for organized clouds)
    pub height: u32,

    /// Whether the point cloud is organized (structured) or unorganized
    pub is_organized: bool,

    /// Sensor origin position [x, y, z]
    pub sensor_origin: [f32; 3],

    /// Sensor orientation quaternion [w, x, y, z]
    pub sensor_orientation: [f32; 4],

    /// Additional custom fields
    pub custom_fields: std::collections::HashMap<String, String>,
}

impl Metadata {
    /// Create new metadata for an unorganized point cloud
    pub fn new_unorganized(point_count: usize) -> Self {
        Self {
            width: point_count as u32,
            height: 1,
            is_organized: false,
            sensor_origin: [0.0, 0.0, 0.0],
            sensor_orientation: [1.0, 0.0, 0.0, 0.0], // Identity quaternion
            custom_fields: std::collections::HashMap::new(),
        }
    }

    /// Create new metadata for an organized point cloud
    pub fn new_organized(width: u32, height: u32) -> Self {
        Self {
            width,
            height,
            is_organized: true,
            sensor_origin: [0.0, 0.0, 0.0],
            sensor_orientation: [1.0, 0.0, 0.0, 0.0], // Identity quaternion
            custom_fields: std::collections::HashMap::new(),
        }
    }

    /// Get the total number of points
    pub fn point_count(&self) -> usize {
        (self.width * self.height) as usize
    }

    /// Set sensor origin
    pub fn with_sensor_origin(mut self, origin: [f32; 3]) -> Self {
        self.sensor_origin = origin;
        self
    }

    /// Set sensor orientation
    pub fn with_sensor_orientation(mut self, orientation: [f32; 4]) -> Self {
        self.sensor_orientation = orientation;
        self
    }

    /// Add a custom field
    pub fn with_custom_field<K, V>(mut self, key: K, value: V) -> Self
    where
        K: Into<String>,
        V: Into<String>,
    {
        self.custom_fields.insert(key.into(), value.into());
        self
    }

    /// Get a custom field value
    pub fn get_custom_field(&self, key: &str) -> Option<&String> {
        self.custom_fields.get(key)
    }

    /// Check if the point cloud is dense (no invalid points)
    pub fn is_dense(&self) -> bool {
        self.custom_fields
            .get("dense")
            .map(|v| v == "true")
            .unwrap_or(true)
    }

    /// Set whether the point cloud is dense
    pub fn set_dense(&mut self, dense: bool) {
        self.custom_fields
            .insert("dense".to_string(), dense.to_string());
    }
}

impl Default for Metadata {
    fn default() -> Self {
        Self::new_unorganized(0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_metadata_creation() {
        let meta = Metadata::new_unorganized(1000);
        assert_eq!(meta.width, 1000);
        assert_eq!(meta.height, 1);
        assert!(!meta.is_organized);
        assert_eq!(meta.point_count(), 1000);
    }

    #[test]
    fn test_organized_metadata() {
        let meta = Metadata::new_organized(640, 480);
        assert_eq!(meta.width, 640);
        assert_eq!(meta.height, 480);
        assert!(meta.is_organized);
        assert_eq!(meta.point_count(), 640 * 480);
    }

    #[test]
    fn test_custom_fields() {
        let meta = Metadata::new_unorganized(100)
            .with_custom_field("source", "kinect")
            .with_custom_field("timestamp", "2023-01-01");

        assert_eq!(meta.get_custom_field("source"), Some(&"kinect".to_string()));
        assert_eq!(
            meta.get_custom_field("timestamp"),
            Some(&"2023-01-01".to_string())
        );
        assert_eq!(meta.get_custom_field("nonexistent"), None);
    }
}
