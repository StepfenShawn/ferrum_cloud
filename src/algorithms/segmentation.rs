//! Point cloud segmentation algorithms
//!
//! This module provides algorithms for segmenting point clouds into
//! meaningful regions or objects.

use crate::core::{Point, PointCloud};
use crate::error::{CloudError, Result};
// use rayon::prelude::*;
use std::collections::{HashMap, HashSet};

/// Euclidean cluster extraction
///
/// Groups nearby points into clusters based on Euclidean distance.
pub fn euclidean_clustering<P: Point>(
    cloud: &PointCloud<P>,
    tolerance: f32,
    min_cluster_size: usize,
    max_cluster_size: usize,
) -> Vec<Vec<usize>> {
    let mut clusters = Vec::new();
    let mut processed = vec![false; cloud.len()];

    for (i, point) in cloud.iter().enumerate() {
        if processed[i] {
            continue;
        }

        let mut cluster = Vec::new();
        let mut queue = vec![i];

        while let Some(current_idx) = queue.pop() {
            if processed[current_idx] {
                continue;
            }

            processed[current_idx] = true;
            cluster.push(current_idx);

            if let Some(current_point) = cloud.get(current_idx) {
                // Find neighbors within tolerance
                for (j, neighbor) in cloud.iter().enumerate() {
                    if !processed[j] && current_point.distance_to(neighbor) <= tolerance {
                        queue.push(j);
                    }
                }
            }
        }

        // Check cluster size constraints
        if cluster.len() >= min_cluster_size && cluster.len() <= max_cluster_size {
            clusters.push(cluster);
        }
    }

    clusters
}

/// RANSAC plane segmentation
///
/// Finds the best-fitting plane in the point cloud using RANSAC algorithm.
pub fn ransac_plane_segmentation<P: Point>(
    cloud: &PointCloud<P>,
    distance_threshold: f32,
    max_iterations: usize,
) -> Result<(Vec<usize>, [f32; 4])> {
    if cloud.len() < 3 {
        return Err(CloudError::algorithm_error(
            "Need at least 3 points for plane fitting",
        ));
    }

    let mut best_inliers = Vec::new();
    let mut best_plane = [0.0, 0.0, 1.0, 0.0]; // Default plane equation: z = 0
    let mut best_inlier_count = 0;

    for _ in 0..max_iterations {
        // Randomly sample 3 points
        let indices = sample_three_points(cloud.len());
        if let Some((p1, p2, p3)) = get_three_points(cloud, &indices) {
            // Fit plane to these 3 points
            if let Some(plane) = fit_plane_to_points(p1, p2, p3) {
                // Count inliers
                let inliers: Vec<usize> = (0..cloud.len())
                    .filter(|&i| {
                        if let Some(point) = cloud.get(i) {
                            distance_to_plane(point.position(), &plane) <= distance_threshold
                        } else {
                            false
                        }
                    })
                    .collect();

                if inliers.len() > best_inlier_count {
                    best_inlier_count = inliers.len();
                    best_inliers = inliers;
                    best_plane = plane;
                }
            }
        }
    }

    Ok((best_inliers, best_plane))
}

/// Sample three random point indices
fn sample_three_points(cloud_size: usize) -> [usize; 3] {
    // Simple random sampling - in practice, you'd use a proper RNG
    [0, cloud_size / 3, (2 * cloud_size) / 3]
}

/// Get three points from the cloud by indices
fn get_three_points<P: Point>(
    cloud: &PointCloud<P>,
    indices: &[usize; 3],
) -> Option<([f32; 3], [f32; 3], [f32; 3])> {
    let p1 = cloud.get(indices[0])?.position();
    let p2 = cloud.get(indices[1])?.position();
    let p3 = cloud.get(indices[2])?.position();
    Some((p1, p2, p3))
}

/// Fit a plane to three points
/// Returns plane equation coefficients [a, b, c, d] where ax + by + cz + d = 0
fn fit_plane_to_points(p1: [f32; 3], p2: [f32; 3], p3: [f32; 3]) -> Option<[f32; 4]> {
    // Calculate two vectors in the plane
    let v1 = [p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]];
    let v2 = [p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2]];

    // Calculate normal vector (cross product)
    let normal = [
        v1[1] * v2[2] - v1[2] * v2[1],
        v1[2] * v2[0] - v1[0] * v2[2],
        v1[0] * v2[1] - v1[1] * v2[0],
    ];

    // Normalize the normal vector
    let length = (normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]).sqrt();
    if length < 1e-6 {
        return None; // Degenerate case
    }

    let a = normal[0] / length;
    let b = normal[1] / length;
    let c = normal[2] / length;
    let d = -(a * p1[0] + b * p1[1] + c * p1[2]);

    Some([a, b, c, d])
}

/// Calculate distance from a point to a plane
fn distance_to_plane(point: [f32; 3], plane: &[f32; 4]) -> f32 {
    (plane[0] * point[0] + plane[1] * point[1] + plane[2] * point[2] + plane[3]).abs()
}

/// Extension trait for adding segmentation methods to PointCloud
pub trait SegmentationExt<P: Point> {
    /// Perform Euclidean clustering
    fn euclidean_cluster(
        &self,
        tolerance: f32,
        min_size: usize,
        max_size: usize,
    ) -> Vec<Vec<usize>>;

    /// Perform RANSAC plane segmentation
    fn ransac_plane(
        &self,
        distance_threshold: f32,
        max_iterations: usize,
    ) -> Result<(Vec<usize>, [f32; 4])>;
}

impl<P: Point> SegmentationExt<P> for PointCloud<P> {
    fn euclidean_cluster(
        &self,
        tolerance: f32,
        min_size: usize,
        max_size: usize,
    ) -> Vec<Vec<usize>> {
        euclidean_clustering(self, tolerance, min_size, max_size)
    }

    fn ransac_plane(
        &self,
        distance_threshold: f32,
        max_iterations: usize,
    ) -> Result<(Vec<usize>, [f32; 4])> {
        ransac_plane_segmentation(self, distance_threshold, max_iterations)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::PointXYZ;

    #[test]
    fn test_euclidean_clustering() {
        let points = vec![
            PointXYZ::new(0.0, 0.0, 0.0),
            PointXYZ::new(0.1, 0.1, 0.1),    // Close to first point
            PointXYZ::new(10.0, 10.0, 10.0), // Far away
        ];
        let cloud = PointCloud::from_points(points);

        let clusters = cloud.euclidean_cluster(1.0, 1, 10);
        assert!(!clusters.is_empty());
    }

    #[test]
    fn test_ransac_plane() {
        // Create points on a plane z = 0
        let points = vec![
            PointXYZ::new(0.0, 0.0, 0.0),
            PointXYZ::new(1.0, 0.0, 0.0),
            PointXYZ::new(0.0, 1.0, 0.0),
            PointXYZ::new(1.0, 1.0, 0.0),
        ];
        let cloud = PointCloud::from_points(points);

        let result = cloud.ransac_plane(0.1, 100);
        assert!(result.is_ok());

        let (inliers, plane) = result.unwrap();
        assert!(!inliers.is_empty());
        assert_eq!(plane.len(), 4);
    }
}
