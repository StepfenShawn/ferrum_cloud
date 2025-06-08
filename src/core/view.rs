//! Point cloud view for zero-copy operations
//!
//! This module provides PointCloudView which allows zero-copy access to
//! point cloud data through borrowing.

use crate::core::{Metadata, Point};
use rayon::prelude::*;

/// Zero-copy view into a point cloud
///
/// This structure provides read-only access to point cloud data without
/// taking ownership, enabling efficient processing pipelines.
#[derive(Debug)]
pub struct PointCloudView<'a, P: Point> {
    /// Slice of points
    points: &'a [P],

    /// Reference to metadata
    metadata: &'a Metadata,
}

impl<'a, P: Point> PointCloudView<'a, P> {
    /// Create a new point cloud view
    pub fn new(points: &'a [P], metadata: &'a Metadata) -> Self {
        Self { points, metadata }
    }

    /// Get the number of points
    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// Check if the view is empty
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Get a reference to the points slice
    pub fn points(&self) -> &'a [P] {
        self.points
    }

    /// Get a reference to the metadata
    pub fn metadata(&self) -> &'a Metadata {
        self.metadata
    }

    /// Get a point by index
    pub fn get(&self, index: usize) -> Option<&'a P> {
        self.points.get(index)
    }

    /// Create an iterator over the points
    pub fn iter(&self) -> std::slice::Iter<'a, P> {
        self.points.iter()
    }

    /// Create a parallel iterator over the points
    pub fn par_iter(&self) -> rayon::slice::Iter<'a, P> {
        self.points.par_iter()
    }

    /// Create a subview with a slice of points
    pub fn slice(&self, range: std::ops::Range<usize>) -> Option<Self> {
        if range.end <= self.points.len() {
            Some(Self {
                points: &self.points[range],
                metadata: self.metadata,
            })
        } else {
            None
        }
    }

    /// Filter points and collect into a new vector
    pub fn filter_collect<F>(&self, predicate: F) -> Vec<P>
    where
        F: Fn(&P) -> bool + Send + Sync,
        P: Clone,
    {
        self.points
            .par_iter()
            .filter(|p| predicate(p))
            .cloned()
            .collect()
    }

    /// Map points and collect into a new vector
    pub fn map_collect<F, Q>(&self, mapper: F) -> Vec<Q>
    where
        F: Fn(&P) -> Q + Send + Sync,
        Q: Send,
    {
        self.points.par_iter().map(mapper).collect()
    }

    /// Calculate the bounding box of the view
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

    /// Calculate the centroid of the view
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

    /// Find the closest point to a query point
    pub fn find_closest(&self, query: &P) -> Option<(usize, &'a P, f32)> {
        if self.is_empty() {
            return None;
        }

        let (index, point, distance_sq) = self
            .points
            .par_iter()
            .enumerate()
            .map(|(i, p)| (i, p, query.distance_squared_to(p)))
            .min_by(|(_, _, d1), (_, _, d2)| d1.partial_cmp(d2).unwrap())?;

        Some((index, point, distance_sq.sqrt()))
    }

    /// Count points that satisfy a predicate
    pub fn count_where<F>(&self, predicate: F) -> usize
    where
        F: Fn(&P) -> bool + Send + Sync,
    {
        self.points.par_iter().filter(|p| predicate(p)).count()
    }
}

impl<'a, P: Point> Clone for PointCloudView<'a, P> {
    fn clone(&self) -> Self {
        Self {
            points: self.points,
            metadata: self.metadata,
        }
    }
}

impl<'a, P: Point> Copy for PointCloudView<'a, P> {}

impl<'a, P: Point> IntoIterator for PointCloudView<'a, P> {
    type Item = &'a P;
    type IntoIter = std::slice::Iter<'a, P>;

    fn into_iter(self) -> Self::IntoIter {
        self.points.iter()
    }
}

impl<'a, P: Point> IntoIterator for &PointCloudView<'a, P> {
    type Item = &'a P;
    type IntoIter = std::slice::Iter<'a, P>;

    fn into_iter(self) -> Self::IntoIter {
        self.points.iter()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::{PointCloud, PointXYZ};

    #[test]
    fn test_point_cloud_view_creation() {
        let points = vec![PointXYZ::new(0.0, 0.0, 0.0), PointXYZ::new(1.0, 1.0, 1.0)];
        let cloud = PointCloud::from_points(points);
        let view = PointCloudView::new(cloud.points(), cloud.metadata());

        assert_eq!(view.len(), 2);
        assert!(!view.is_empty());
    }

    #[test]
    fn test_view_slice() {
        let points = vec![
            PointXYZ::new(0.0, 0.0, 0.0),
            PointXYZ::new(1.0, 1.0, 1.0),
            PointXYZ::new(2.0, 2.0, 2.0),
        ];
        let cloud = PointCloud::from_points(points);
        let view = PointCloudView::new(cloud.points(), cloud.metadata());

        let subview = view.slice(1..3).unwrap();
        assert_eq!(subview.len(), 2);
        assert_eq!(subview.get(0).unwrap().position(), [1.0, 1.0, 1.0]);
    }

    #[test]
    fn test_view_operations() {
        let points = vec![
            PointXYZ::new(0.0, 0.0, 0.0),
            PointXYZ::new(1.0, 1.0, 1.0),
            PointXYZ::new(2.0, 2.0, 2.0),
        ];
        let cloud = PointCloud::from_points(points);
        let view = PointCloudView::new(cloud.points(), cloud.metadata());

        let filtered = view.filter_collect(|p| p.x() > 0.5);
        assert_eq!(filtered.len(), 2);

        let count = view.count_where(|p| p.x() >= 1.0);
        assert_eq!(count, 2);
    }
}
