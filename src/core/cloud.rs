//! Point cloud container
//!
//! This module provides the main PointCloud container that owns point data
//! and provides methods for manipulation and processing.

use crate::core::{Metadata, Point};
use crate::error::{CloudError, Result};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use std::sync::Arc;

/// Main point cloud container that owns point data
///
/// This structure provides ownership-based point cloud management with
/// efficient parallel processing capabilities.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PointCloud<P: Point> {
    /// Vector of points
    points: Vec<P>,

    /// Metadata associated with the point cloud
    metadata: Metadata,
}

impl<P: Point> PointCloud<P> {
    /// Create a new empty point cloud
    pub fn new() -> Self {
        Self {
            points: Vec::new(),
            metadata: Metadata::default(),
        }
    }

    /// Create a point cloud from a vector of points
    pub fn from_points(points: Vec<P>) -> Self {
        let metadata = Metadata::new_unorganized(points.len());
        Self { points, metadata }
    }

    /// Create a point cloud with specified capacity
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            points: Vec::with_capacity(capacity),
            metadata: Metadata::new_unorganized(0),
        }
    }

    /// Create a point cloud from points and metadata
    pub fn from_points_and_metadata(points: Vec<P>, metadata: Metadata) -> Self {
        Self { points, metadata }
    }

    /// Get the number of points
    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// Check if the point cloud is empty
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Get a reference to the points vector
    pub fn points(&self) -> &[P] {
        &self.points
    }

    /// Get a mutable reference to the points vector
    pub fn points_mut(&mut self) -> &mut Vec<P> {
        &mut self.points
    }

    /// Get a reference to the metadata
    pub fn metadata(&self) -> &Metadata {
        &self.metadata
    }

    /// Get a mutable reference to the metadata
    pub fn metadata_mut(&mut self) -> &mut Metadata {
        &mut self.metadata
    }

    /// Add a point to the cloud
    pub fn push(&mut self, point: P) {
        self.points.push(point);
        self.metadata.width = self.points.len() as u32;
    }

    /// Add multiple points to the cloud
    pub fn extend<I>(&mut self, points: I)
    where
        I: IntoIterator<Item = P>,
    {
        self.points.extend(points);
        self.metadata.width = self.points.len() as u32;
    }

    /// Get a point by index
    pub fn get(&self, index: usize) -> Option<&P> {
        self.points.get(index)
    }

    /// Get a mutable reference to a point by index
    pub fn get_mut(&mut self, index: usize) -> Option<&mut P> {
        self.points.get_mut(index)
    }

    /// Remove a point by index
    pub fn remove(&mut self, index: usize) -> P {
        let point = self.points.remove(index);
        self.metadata.width = self.points.len() as u32;
        point
    }

    /// Clear all points
    pub fn clear(&mut self) {
        self.points.clear();
        self.metadata.width = 0;
    }

    /// Reserve capacity for additional points
    pub fn reserve(&mut self, additional: usize) {
        self.points.reserve(additional);
    }

    /// Shrink the capacity to fit the current number of points
    pub fn shrink_to_fit(&mut self) {
        self.points.shrink_to_fit();
    }

    /// Create an iterator over the points
    pub fn iter(&self) -> std::slice::Iter<P> {
        self.points.iter()
    }

    /// Create a mutable iterator over the points
    pub fn iter_mut(&mut self) -> std::slice::IterMut<P> {
        self.points.iter_mut()
    }

    /// Create a parallel iterator over the points
    pub fn par_iter(&self) -> rayon::slice::Iter<P> {
        self.points.par_iter()
    }

    /// Create a parallel mutable iterator over the points
    pub fn par_iter_mut(&mut self) -> rayon::slice::IterMut<P> {
        self.points.par_iter_mut()
    }

    /// Filter points based on a predicate
    pub fn filter<F>(self, predicate: F) -> Self
    where
        F: Fn(&P) -> bool + Send + Sync,
    {
        let filtered_points: Vec<P> = self
            .points
            .into_par_iter()
            .filter(|p| predicate(p))
            .collect();

        let mut metadata = self.metadata;
        metadata.width = filtered_points.len() as u32;

        Self {
            points: filtered_points,
            metadata,
        }
    }

    /// Transform points using a mapping function
    pub fn map<F, Q>(self, mapper: F) -> PointCloud<Q>
    where
        F: Fn(P) -> Q + Send + Sync,
        Q: Point,
    {
        let mapped_points: Vec<Q> = self.points.into_par_iter().map(mapper).collect();

        PointCloud {
            points: mapped_points,
            metadata: self.metadata,
        }
    }

    /// Calculate the bounding box of the point cloud
    pub fn bounding_box(&self) -> Option<([f32; 3], [f32; 3])> {
        if self.is_empty() {
            return None;
        }

        let (min_vals, max_vals) = self
            .points
            .par_iter()
            .map(|p| {
                let pos = p.position();
                (pos, pos)
            })
            .reduce_with(|(min1, max1), (min2, max2)| {
                (
                    [
                        min1[0].min(min2[0]),
                        min1[1].min(min2[1]),
                        min1[2].min(min2[2]),
                    ],
                    [
                        max1[0].max(max2[0]),
                        max1[1].max(max2[1]),
                        max1[2].max(max2[2]),
                    ],
                )
            })?;

        Some((min_vals, max_vals))
    }

    /// Calculate the centroid of the point cloud
    pub fn centroid(&self) -> Option<[f32; 3]> {
        if self.is_empty() {
            return None;
        }

        let sum = self
            .points
            .par_iter()
            .map(|p| p.position())
            .reduce_with(|acc, pos| [acc[0] + pos[0], acc[1] + pos[1], acc[2] + pos[2]])?;

        let len = self.len() as f32;
        Some([sum[0] / len, sum[1] / len, sum[2] / len])
    }

    /// Crop the point cloud to a bounding box
    pub fn crop(self, min_bounds: [f32; 3], max_bounds: [f32; 3]) -> Self {
        self.filter(|p| {
            let pos = p.position();
            pos[0] >= min_bounds[0]
                && pos[0] <= max_bounds[0]
                && pos[1] >= min_bounds[1]
                && pos[1] <= max_bounds[1]
                && pos[2] >= min_bounds[2]
                && pos[2] <= max_bounds[2]
        })
    }

    /// Convert to shared ownership using Arc
    pub fn into_shared(self) -> Arc<Self> {
        Arc::new(self)
    }
}

impl<P: Point> Default for PointCloud<P> {
    fn default() -> Self {
        Self::new()
    }
}

impl<P: Point> FromIterator<P> for PointCloud<P> {
    fn from_iter<I: IntoIterator<Item = P>>(iter: I) -> Self {
        let points: Vec<P> = iter.into_iter().collect();
        Self::from_points(points)
    }
}

impl<P: Point> IntoIterator for PointCloud<P> {
    type Item = P;
    type IntoIter = std::vec::IntoIter<P>;

    fn into_iter(self) -> Self::IntoIter {
        self.points.into_iter()
    }
}

impl<'a, P: Point> IntoIterator for &'a PointCloud<P> {
    type Item = &'a P;
    type IntoIter = std::slice::Iter<'a, P>;

    fn into_iter(self) -> Self::IntoIter {
        self.points.iter()
    }
}

impl<'a, P: Point> IntoIterator for &'a mut PointCloud<P> {
    type Item = &'a mut P;
    type IntoIter = std::slice::IterMut<'a, P>;

    fn into_iter(self) -> Self::IntoIter {
        self.points.iter_mut()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::PointXYZ;

    #[test]
    fn test_point_cloud_creation() {
        let cloud = PointCloud::<PointXYZ>::new();
        assert!(cloud.is_empty());
        assert_eq!(cloud.len(), 0);
    }

    #[test]
    fn test_point_cloud_from_points() {
        let points = vec![PointXYZ::new(0.0, 0.0, 0.0), PointXYZ::new(1.0, 1.0, 1.0)];
        let cloud = PointCloud::from_points(points);
        assert_eq!(cloud.len(), 2);
        assert!(!cloud.is_empty());
    }

    #[test]
    fn test_point_cloud_operations() {
        let mut cloud = PointCloud::new();
        cloud.push(PointXYZ::new(1.0, 2.0, 3.0));
        cloud.push(PointXYZ::new(4.0, 5.0, 6.0));

        assert_eq!(cloud.len(), 2);
        assert_eq!(cloud.get(0).unwrap().position(), [1.0, 2.0, 3.0]);
        assert_eq!(cloud.get(1).unwrap().position(), [4.0, 5.0, 6.0]);
    }

    #[test]
    fn test_bounding_box() {
        let points = vec![
            PointXYZ::new(-1.0, -2.0, -3.0),
            PointXYZ::new(1.0, 2.0, 3.0),
            PointXYZ::new(0.0, 0.0, 0.0),
        ];
        let cloud = PointCloud::from_points(points);

        let (min_bounds, max_bounds) = cloud.bounding_box().unwrap();
        assert_eq!(min_bounds, [-1.0, -2.0, -3.0]);
        assert_eq!(max_bounds, [1.0, 2.0, 3.0]);
    }

    #[test]
    fn test_centroid() {
        let points = vec![PointXYZ::new(0.0, 0.0, 0.0), PointXYZ::new(2.0, 2.0, 2.0)];
        let cloud = PointCloud::from_points(points);

        let centroid = cloud.centroid().unwrap();
        assert_eq!(centroid, [1.0, 1.0, 1.0]);
    }

    #[test]
    fn test_filter() {
        let points = vec![
            PointXYZ::new(0.0, 0.0, 0.0),
            PointXYZ::new(1.0, 1.0, 1.0),
            PointXYZ::new(2.0, 2.0, 2.0),
        ];
        let cloud = PointCloud::from_points(points);

        let filtered = cloud.filter(|p| p.x() > 0.5);
        assert_eq!(filtered.len(), 2);
    }
}
