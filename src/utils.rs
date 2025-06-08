//! Utility functions and helpers
//!
//! This module provides various utility functions for point cloud processing,
//! including mathematical operations, conversions, and helper functions.

use crate::core::Point;
use std::f32::consts::PI;

/// Mathematical constants and utility functions
pub mod math {
    use super::*;

    /// Convert degrees to radians
    pub fn deg_to_rad(degrees: f32) -> f32 {
        degrees * PI / 180.0
    }

    /// Convert radians to degrees
    pub fn rad_to_deg(radians: f32) -> f32 {
        radians * 180.0 / PI
    }

    /// Clamp a value between min and max
    pub fn clamp(value: f32, min: f32, max: f32) -> f32 {
        if value < min {
            min
        } else if value > max {
            max
        } else {
            value
        }
    }

    /// Linear interpolation between two values
    pub fn lerp(a: f32, b: f32, t: f32) -> f32 {
        a + (b - a) * t
    }

    /// Calculate the dot product of two 3D vectors
    pub fn dot_product(a: [f32; 3], b: [f32; 3]) -> f32 {
        a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
    }

    /// Calculate the cross product of two 3D vectors
    pub fn cross_product(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    }

    /// Normalize a 3D vector
    pub fn normalize(v: [f32; 3]) -> [f32; 3] {
        let length = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
        if length > 1e-6 {
            [v[0] / length, v[1] / length, v[2] / length]
        } else {
            [0.0, 0.0, 0.0]
        }
    }

    /// Calculate the magnitude of a 3D vector
    pub fn magnitude(v: [f32; 3]) -> f32 {
        (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
    }
}

/// Color conversion utilities
pub mod color {
    /// Convert RGB values (0-255) to normalized floats (0.0-1.0)
    pub fn rgb_to_normalized(r: u8, g: u8, b: u8) -> [f32; 3] {
        [r as f32 / 255.0, g as f32 / 255.0, b as f32 / 255.0]
    }

    /// Convert normalized RGB floats (0.0-1.0) to RGB values (0-255)
    pub fn normalized_to_rgb(rgb: [f32; 3]) -> [u8; 3] {
        [
            (rgb[0] * 255.0).clamp(0.0, 255.0) as u8,
            (rgb[1] * 255.0).clamp(0.0, 255.0) as u8,
            (rgb[2] * 255.0).clamp(0.0, 255.0) as u8,
        ]
    }

    /// Convert HSV to RGB
    pub fn hsv_to_rgb(h: f32, s: f32, v: f32) -> [f32; 3] {
        let c = v * s;
        let x = c * (1.0 - ((h / 60.0) % 2.0 - 1.0).abs());
        let m = v - c;

        let (r, g, b) = if h < 60.0 {
            (c, x, 0.0)
        } else if h < 120.0 {
            (x, c, 0.0)
        } else if h < 180.0 {
            (0.0, c, x)
        } else if h < 240.0 {
            (0.0, x, c)
        } else if h < 300.0 {
            (x, 0.0, c)
        } else {
            (c, 0.0, x)
        };

        [r + m, g + m, b + m]
    }
}

/// Point cloud statistics and analysis utilities
pub mod stats {
    use super::*;
    use crate::core::PointCloud;

    /// Calculate basic statistics for a point cloud
    pub fn calculate_statistics<P: Point>(cloud: &PointCloud<P>) -> CloudStatistics {
        if cloud.is_empty() {
            return CloudStatistics::default();
        }

        let positions: Vec<[f32; 3]> = cloud.iter().map(|p| p.position()).collect();

        // Calculate mean
        let sum = positions.iter().fold([0.0; 3], |acc, pos| {
            [acc[0] + pos[0], acc[1] + pos[1], acc[2] + pos[2]]
        });
        let count = positions.len() as f32;
        let mean = [sum[0] / count, sum[1] / count, sum[2] / count];

        // Calculate variance and standard deviation
        let variance = positions.iter().fold([0.0; 3], |acc, pos| {
            let diff = [pos[0] - mean[0], pos[1] - mean[1], pos[2] - mean[2]];
            [
                acc[0] + diff[0] * diff[0],
                acc[1] + diff[1] * diff[1],
                acc[2] + diff[2] * diff[2],
            ]
        });
        let variance = [
            variance[0] / count,
            variance[1] / count,
            variance[2] / count,
        ];
        let std_dev = [variance[0].sqrt(), variance[1].sqrt(), variance[2].sqrt()];

        // Find min and max
        let mut min = positions[0];
        let mut max = positions[0];
        for pos in &positions {
            for i in 0..3 {
                min[i] = min[i].min(pos[i]);
                max[i] = max[i].max(pos[i]);
            }
        }

        CloudStatistics {
            count: cloud.len(),
            mean,
            std_dev,
            variance,
            min,
            max,
        }
    }

    /// Statistical information about a point cloud
    #[derive(Debug, Clone)]
    pub struct CloudStatistics {
        pub count: usize,
        pub mean: [f32; 3],
        pub std_dev: [f32; 3],
        pub variance: [f32; 3],
        pub min: [f32; 3],
        pub max: [f32; 3],
    }

    impl Default for CloudStatistics {
        fn default() -> Self {
            Self {
                count: 0,
                mean: [0.0; 3],
                std_dev: [0.0; 3],
                variance: [0.0; 3],
                min: [0.0; 3],
                max: [0.0; 3],
            }
        }
    }
}

/// Performance measurement utilities
pub mod perf {
    use std::time::{Duration, Instant};

    /// Simple timer for measuring execution time
    pub struct Timer {
        start: Instant,
    }

    impl Timer {
        /// Create a new timer and start measuring
        pub fn new() -> Self {
            Self {
                start: Instant::now(),
            }
        }

        /// Get elapsed time since timer creation
        pub fn elapsed(&self) -> Duration {
            self.start.elapsed()
        }

        /// Get elapsed time in milliseconds
        pub fn elapsed_ms(&self) -> f64 {
            self.elapsed().as_secs_f64() * 1000.0
        }

        /// Reset the timer
        pub fn reset(&mut self) {
            self.start = Instant::now();
        }
    }

    impl Default for Timer {
        fn default() -> Self {
            Self::new()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::{PointCloud, PointXYZ};

    #[test]
    fn test_math_functions() {
        assert!((math::deg_to_rad(180.0) - PI).abs() < 1e-6);
        assert!((math::rad_to_deg(PI) - 180.0).abs() < 1e-6);

        assert_eq!(math::clamp(5.0, 0.0, 10.0), 5.0);
        assert_eq!(math::clamp(-1.0, 0.0, 10.0), 0.0);
        assert_eq!(math::clamp(15.0, 0.0, 10.0), 10.0);

        assert_eq!(math::lerp(0.0, 10.0, 0.5), 5.0);

        let a = [1.0, 0.0, 0.0];
        let b = [0.0, 1.0, 0.0];
        assert_eq!(math::dot_product(a, b), 0.0);
        assert_eq!(math::cross_product(a, b), [0.0, 0.0, 1.0]);

        let v = [3.0, 4.0, 0.0];
        assert_eq!(math::magnitude(v), 5.0);

        let normalized = math::normalize(v);
        assert!((math::magnitude(normalized) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_color_conversion() {
        let rgb = color::rgb_to_normalized(255, 128, 0);
        assert_eq!(rgb, [1.0, 0.5019608, 0.0]);

        let back = color::normalized_to_rgb(rgb);
        assert_eq!(back, [255, 128, 0]);
    }

    #[test]
    fn test_statistics() {
        let points = vec![
            PointXYZ::new(0.0, 0.0, 0.0),
            PointXYZ::new(1.0, 1.0, 1.0),
            PointXYZ::new(2.0, 2.0, 2.0),
        ];
        let cloud = PointCloud::from_points(points);

        let stats = stats::calculate_statistics(&cloud);
        assert_eq!(stats.count, 3);
        assert_eq!(stats.mean, [1.0, 1.0, 1.0]);
        assert_eq!(stats.min, [0.0, 0.0, 0.0]);
        assert_eq!(stats.max, [2.0, 2.0, 2.0]);
    }

    #[test]
    fn test_timer() {
        let timer = perf::Timer::new();
        std::thread::sleep(std::time::Duration::from_millis(10));
        let elapsed = timer.elapsed_ms();
        assert!(elapsed >= 10.0);
    }
}
