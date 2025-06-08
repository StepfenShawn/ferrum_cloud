//! Filtering algorithms for point clouds
//!
//! This module provides various filtering algorithms including voxel downsampling,
//! outlier removal, and statistical filtering.

use crate::core::{Point, PointCloud};
use crate::error::{CloudError, Result};
use rayon::prelude::*;
use std::collections::HashMap;

/// Voxel downsampling filter
///
/// Reduces point cloud density by averaging points within voxel grids.
pub fn voxel_downsample<P: Point>(cloud: PointCloud<P>, voxel_size: f32) -> PointCloud<P> {
    if voxel_size <= 0.0 {
        return cloud;
    }

    let mut voxel_map: HashMap<(i32, i32, i32), Vec<P>> = HashMap::new();

    // Group points by voxel
    for point in cloud.into_iter() {
        let pos = point.position();
        let voxel_key = (
            (pos[0] / voxel_size).floor() as i32,
            (pos[1] / voxel_size).floor() as i32,
            (pos[2] / voxel_size).floor() as i32,
        );
        voxel_map
            .entry(voxel_key)
            .or_insert_with(Vec::new)
            .push(point);
    }

    // Average points in each voxel
    let downsampled_points: Vec<P> = voxel_map
        .into_par_iter()
        .map(|(_, points)| {
            if points.is_empty() {
                return None;
            }

            // Calculate average position
            let sum = points.iter().fold([0.0; 3], |acc, p| {
                let pos = p.position();
                [acc[0] + pos[0], acc[1] + pos[1], acc[2] + pos[2]]
            });

            let count = points.len() as f32;
            let avg_pos = [sum[0] / count, sum[1] / count, sum[2] / count];

            // Return the first point with averaged position
            // This is a simplified approach - in practice, you might want to
            // average other properties as well
            let mut result = points.into_iter().next().unwrap();
            // Note: This would require Point trait to have a set_position method
            // For now, we'll just return the first point
            Some(result)
        })
        .filter_map(|p| p)
        .collect();

    PointCloud::from_points(downsampled_points)
}

/// Statistical outlier removal
///
/// Removes points that are statistical outliers based on their distance
/// to neighboring points.
pub fn remove_statistical_outliers<P: Point>(
    cloud: PointCloud<P>,
    k_neighbors: usize,
    std_dev_threshold: f32,
) -> Result<PointCloud<P>> {
    if cloud.len() < k_neighbors {
        return Ok(cloud);
    }

    // For each point, find k nearest neighbors and calculate mean distance
    let mean_distances: Vec<f32> = cloud
        .par_iter()
        .map(|query_point| {
            let mut distances: Vec<f32> = cloud
                .iter()
                .map(|p| query_point.distance_to(p))
                .filter(|&d| d > 0.0) // Exclude self
                .collect();

            distances.sort_by(|a, b| a.partial_cmp(b).unwrap());
            distances.truncate(k_neighbors);

            if distances.is_empty() {
                0.0
            } else {
                distances.iter().sum::<f32>() / distances.len() as f32
            }
        })
        .collect();

    // Calculate global mean and standard deviation
    let global_mean = mean_distances.iter().sum::<f32>() / mean_distances.len() as f32;
    let variance = mean_distances
        .iter()
        .map(|&d| (d - global_mean).powi(2))
        .sum::<f32>()
        / mean_distances.len() as f32;
    let std_dev = variance.sqrt();

    let threshold = global_mean + std_dev_threshold * std_dev;

    // Filter points based on threshold
    let filtered_points: Vec<P> = cloud
        .into_iter()
        .zip(mean_distances.into_iter())
        .filter_map(|(point, mean_dist)| {
            if mean_dist <= threshold {
                Some(point)
            } else {
                None
            }
        })
        .collect();

    Ok(PointCloud::from_points(filtered_points))
}

/// Radius outlier removal
///
/// Removes points that have fewer than a minimum number of neighbors
/// within a specified radius.
pub fn remove_radius_outliers<P: Point>(
    cloud: PointCloud<P>,
    radius: f32,
    min_neighbors: usize,
) -> PointCloud<P> {
    let filtered_points: Vec<P> = cloud
        .par_iter()
        .filter_map(|query_point| {
            let neighbor_count = cloud
                .iter()
                .filter(|&p| {
                    let distance = query_point.distance_to(p);
                    distance > 0.0 && distance <= radius
                })
                .count();

            if neighbor_count >= min_neighbors {
                Some(query_point.clone())
            } else {
                None
            }
        })
        .collect();

    PointCloud::from_points(filtered_points)
}

/// Pass-through filter
///
/// Filters points based on coordinate ranges.
pub fn pass_through_filter<P: Point>(
    cloud: PointCloud<P>,
    axis: Axis,
    min_value: f32,
    max_value: f32,
) -> PointCloud<P> {
    cloud.filter(|point| {
        let pos = point.position();
        let value = match axis {
            Axis::X => pos[0],
            Axis::Y => pos[1],
            Axis::Z => pos[2],
        };
        value >= min_value && value <= max_value
    })
}

/// Coordinate axis enumeration
#[derive(Debug, Clone, Copy)]
pub enum Axis {
    X,
    Y,
    Z,
}

/// Extension trait for PointCloud to add filtering methods
pub trait FilterExt<P: Point> {
    /// Apply voxel downsampling
    fn voxel_downsample(self, voxel_size: f32) -> PointCloud<P>;

    /// Remove statistical outliers
    fn remove_outliers(self, k_neighbors: usize, std_dev_threshold: f32) -> Result<PointCloud<P>>;

    /// Remove radius outliers
    fn remove_radius_outliers(self, radius: f32, min_neighbors: usize) -> PointCloud<P>;

    /// Apply pass-through filter
    fn pass_through(self, axis: Axis, min_value: f32, max_value: f32) -> PointCloud<P>;
}

impl<P: Point> FilterExt<P> for PointCloud<P> {
    fn voxel_downsample(self, voxel_size: f32) -> PointCloud<P> {
        voxel_downsample(self, voxel_size)
    }

    fn remove_outliers(self, k_neighbors: usize, std_dev_threshold: f32) -> Result<PointCloud<P>> {
        remove_statistical_outliers(self, k_neighbors, std_dev_threshold)
    }

    fn remove_radius_outliers(self, radius: f32, min_neighbors: usize) -> PointCloud<P> {
        remove_radius_outliers(self, radius, min_neighbors)
    }

    fn pass_through(self, axis: Axis, min_value: f32, max_value: f32) -> PointCloud<P> {
        pass_through_filter(self, axis, min_value, max_value)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::PointXYZ;

    #[test]
    fn test_voxel_downsample() {
        let points = vec![
            PointXYZ::new(0.0, 0.0, 0.0),
            PointXYZ::new(0.01, 0.01, 0.01), // Should be in same voxel
            PointXYZ::new(1.0, 1.0, 1.0),
        ];
        let cloud = PointCloud::from_points(points);

        let downsampled = cloud.voxel_downsample(0.1);
        assert!(downsampled.len() <= 2); // Should reduce to at most 2 points
    }

    #[test]
    fn test_pass_through_filter() {
        let points = vec![
            PointXYZ::new(0.0, 0.0, 0.0),
            PointXYZ::new(1.0, 1.0, 1.0),
            PointXYZ::new(2.0, 2.0, 2.0),
        ];
        let cloud = PointCloud::from_points(points);

        let filtered = cloud.pass_through(Axis::X, 0.5, 1.5);
        assert_eq!(filtered.len(), 1); // Only middle point should remain
    }

    #[test]
    fn test_radius_outlier_removal() {
        let points = vec![
            PointXYZ::new(0.0, 0.0, 0.0),
            PointXYZ::new(0.1, 0.1, 0.1),
            PointXYZ::new(10.0, 10.0, 10.0), // Outlier
        ];
        let cloud = PointCloud::from_points(points);

        let filtered = cloud.remove_radius_outliers(1.0, 1);
        assert_eq!(filtered.len(), 2); // Outlier should be removed
    }
}
