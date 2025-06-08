//! KD-tree implementation for efficient nearest neighbor search
//!
//! This module provides a KD-tree data structure optimized for 3D point cloud
//! nearest neighbor queries.

use crate::core::Point;
use crate::error::{CloudError, Result};

/// KD-tree for efficient spatial queries
///
/// This is a simplified implementation. A production KD-tree would include
/// more optimizations and better balancing algorithms.
pub struct KdTree<P: Point> {
    root: Option<Box<KdNode<P>>>,
}

/// Node in the KD-tree
struct KdNode<P: Point> {
    point: P,
    axis: usize, // 0=x, 1=y, 2=z
    left: Option<Box<KdNode<P>>>,
    right: Option<Box<KdNode<P>>>,
}

impl<P: Point> KdTree<P> {
    /// Create a new empty KD-tree
    pub fn new() -> Self {
        Self { root: None }
    }

    /// Build a KD-tree from a slice of points
    pub fn build(points: &[P]) -> Self {
        let mut points_copy = points.to_vec();
        let root = Self::build_recursive(&mut points_copy, 0);
        Self { root }
    }

    /// Recursively build the KD-tree
    fn build_recursive(points: &mut [P], depth: usize) -> Option<Box<KdNode<P>>> {
        if points.is_empty() {
            return None;
        }

        let axis = depth % 3;

        // Sort points by the current axis
        points.sort_by(|a, b| a.position()[axis].partial_cmp(&b.position()[axis]).unwrap());

        let median = points.len() / 2;
        let point = points[median].clone();

        let (left_points, right_points) = points.split_at_mut(median);
        let right_points = &mut right_points[1..]; // Skip the median point

        let left = Self::build_recursive(left_points, depth + 1);
        let right = Self::build_recursive(right_points, depth + 1);

        Some(Box::new(KdNode {
            point,
            axis,
            left,
            right,
        }))
    }

    /// Calculate squared Euclidean distance between two 3D points
    fn distance_squared(p1: &[f32; 3], p2: &[f32; 3]) -> f32 {
        let dx = p1[0] - p2[0];
        let dy = p1[1] - p2[1];
        let dz = p1[2] - p2[2];
        dx * dx + dy * dy + dz * dz
    }

    /// Find the nearest neighbor to a query point
    pub fn nearest_neighbor(&self, query: &P) -> Option<&P> {
        self.root.as_ref().map(|root| {
            let mut best_point = &root.point;
            let mut best_distance_squared = f32::INFINITY;

            Self::nearest_neighbor_recursive(
                root,
                query,
                &mut best_point,
                &mut best_distance_squared,
                0,
            );

            best_point
        })
    }

    /// Recursive helper for nearest neighbor search
    fn nearest_neighbor_recursive<'a>(
        node: &'a KdNode<P>,
        query: &P,
        best_point: &mut &'a P,
        best_distance_squared: &mut f32,
        depth: usize,
    ) {
        let distance_squared = Self::distance_squared(&node.point.position(), &query.position());

        if distance_squared < *best_distance_squared {
            *best_distance_squared = distance_squared;
            *best_point = &node.point;
        }

        let axis = depth % 3;
        let query_pos = query.position();
        let node_pos = node.point.position();

        let (primary, secondary) = if query_pos[axis] < node_pos[axis] {
            (&node.left, &node.right)
        } else {
            (&node.right, &node.left)
        };

        if let Some(child) = primary {
            Self::nearest_neighbor_recursive(
                child,
                query,
                best_point,
                best_distance_squared,
                depth + 1,
            );
        }

        let axis_distance = query_pos[axis] - node_pos[axis];
        if axis_distance * axis_distance < *best_distance_squared {
            if let Some(child) = secondary {
                Self::nearest_neighbor_recursive(
                    child,
                    query,
                    best_point,
                    best_distance_squared,
                    depth + 1,
                );
            }
        }
    }

    /// Find all points within a given radius of the query point
    pub fn radius_search(&self, query: &P, radius: f32) -> Vec<(&P, f32)> {
        let mut results = Vec::new();
        let radius_squared = radius * radius;

        if let Some(ref root) = self.root {
            Self::radius_search_recursive(root, query, radius, radius_squared, &mut results, 0);
        }

        results
    }

    /// Recursive helper for radius search
    fn radius_search_recursive<'a>(
        node: &'a KdNode<P>,
        query: &P,
        radius: f32,
        radius_squared: f32,
        results: &mut Vec<(&'a P, f32)>,
        depth: usize,
    ) {
        let distance_squared = Self::distance_squared(&node.point.position(), &query.position());

        if distance_squared <= radius_squared {
            results.push((&node.point, distance_squared));
        }

        let axis = depth % 3;
        let query_pos = query.position();
        let node_pos = node.point.position();

        if let Some(left) = &node.left {
            if query_pos[axis] - radius <= node_pos[axis] {
                Self::radius_search_recursive(
                    left,
                    query,
                    radius,
                    radius_squared,
                    results,
                    depth + 1,
                );
            }
        }

        if let Some(right) = &node.right {
            if query_pos[axis] + radius >= node_pos[axis] {
                Self::radius_search_recursive(
                    right,
                    query,
                    radius,
                    radius_squared,
                    results,
                    depth + 1,
                );
            }
        }
    }

    /// Find the k nearest neighbors to a query point
    pub fn k_nearest(&self, query: &P, k: usize) -> Vec<(&P, f32)> {
        let mut results = Vec::new();

        if let Some(ref root) = self.root {
            Self::k_nearest_recursive(root, query, k, &mut results, 0);
        }

        // Sort by distance and take only k results
        results.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
        results.truncate(k);
        results
    }

    /// Recursive helper for k-nearest search
    fn k_nearest_recursive<'a>(
        node: &'a KdNode<P>,
        query: &P,
        k: usize,
        results: &mut Vec<(&'a P, f32)>,
        depth: usize,
    ) {
        let distance_squared = Self::distance_squared(&node.point.position(), &query.position());

        results.push((&node.point, distance_squared));

        // Keep only the k closest points
        if results.len() > k {
            results.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
            results.truncate(k);
        }

        let axis = depth % 3;
        let query_pos = query.position();
        let node_pos = node.point.position();

        let (primary, secondary) = if query_pos[axis] < node_pos[axis] {
            (&node.left, &node.right)
        } else {
            (&node.right, &node.left)
        };

        if let Some(child) = primary {
            Self::k_nearest_recursive(child, query, k, results, depth + 1);
        }

        // Check if we need to explore the other side
        let worst_distance = if results.len() < k {
            f32::INFINITY
        } else {
            results.iter().map(|(_, d)| *d).fold(0.0, f32::max)
        };

        let axis_distance = query_pos[axis] - node_pos[axis];
        if axis_distance * axis_distance < worst_distance {
            if let Some(child) = secondary {
                Self::k_nearest_recursive(child, query, k, results, depth + 1);
            }
        }
    }
}

impl<P: Point> Default for KdTree<P> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::PointXYZ;

    #[test]
    fn test_kdtree_build() {
        let points = vec![
            PointXYZ::new(0.0, 0.0, 0.0),
            PointXYZ::new(1.0, 1.0, 1.0),
            PointXYZ::new(2.0, 2.0, 2.0),
        ];

        let tree = KdTree::build(&points);
        assert!(tree.root.is_some());
    }

    #[test]
    fn test_nearest_neighbor() {
        let points = vec![
            PointXYZ::new(0.0, 0.0, 0.0),
            PointXYZ::new(1.0, 1.0, 1.0),
            PointXYZ::new(2.0, 2.0, 2.0),
        ];

        let tree = KdTree::build(&points);
        let query = PointXYZ::new(0.1, 0.1, 0.1);

        let result = tree.nearest_neighbor(&query);
        assert!(result.is_some());

        let nearest = result.unwrap();
        assert_eq!(nearest.position(), [0.0, 0.0, 0.0]);
    }

    #[test]
    fn test_radius_search() {
        let points = vec![
            PointXYZ::new(0.0, 0.0, 0.0),
            PointXYZ::new(1.0, 1.0, 1.0),
            PointXYZ::new(10.0, 10.0, 10.0),
        ];

        let tree = KdTree::build(&points);
        let query = PointXYZ::new(0.0, 0.0, 0.0);

        let results = tree.radius_search(&query, 2.0);
        assert_eq!(results.len(), 2); // Should find first two points
    }
}
