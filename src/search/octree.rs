//! Octree implementation for spatial partitioning
//!
//! This module provides an octree data structure for efficient spatial
//! queries and organization of 3D point data.

use crate::core::Point;
use crate::error::{CloudError, Result};

/// Octree for spatial partitioning of 3D points
///
/// This is a simplified implementation. A production octree would include
/// more optimizations and dynamic insertion/deletion capabilities.
pub struct Octree<P: Point> {
    root: Option<Box<OctreeNode<P>>>,
    bounds: BoundingBox,
    max_depth: usize,
    max_points_per_node: usize,
}

/// Node in the octree
struct OctreeNode<P: Point> {
    bounds: BoundingBox,
    points: Vec<P>,
    children: Option<[Box<OctreeNode<P>>; 8]>,
    depth: usize,
}

/// Axis-aligned bounding box
#[derive(Clone, Copy, Debug)]
pub struct BoundingBox {
    pub min: [f32; 3],
    pub max: [f32; 3],
}

impl BoundingBox {
    /// Create a new bounding box
    pub fn new(min: [f32; 3], max: [f32; 3]) -> Self {
        Self { min, max }
    }

    /// Check if a point is inside the bounding box
    pub fn contains(&self, point: [f32; 3]) -> bool {
        point[0] >= self.min[0]
            && point[0] <= self.max[0]
            && point[1] >= self.min[1]
            && point[1] <= self.max[1]
            && point[2] >= self.min[2]
            && point[2] <= self.max[2]
    }

    /// Get the center of the bounding box
    pub fn center(&self) -> [f32; 3] {
        [
            (self.min[0] + self.max[0]) * 0.5,
            (self.min[1] + self.max[1]) * 0.5,
            (self.min[2] + self.max[2]) * 0.5,
        ]
    }

    /// Get the size of the bounding box
    pub fn size(&self) -> [f32; 3] {
        [
            self.max[0] - self.min[0],
            self.max[1] - self.min[1],
            self.max[2] - self.min[2],
        ]
    }
}

impl<P: Point> Octree<P> {
    /// Create a new octree with specified bounds and parameters
    pub fn new(bounds: BoundingBox, max_depth: usize, max_points_per_node: usize) -> Self {
        Self {
            root: None,
            bounds,
            max_depth,
            max_points_per_node,
        }
    }

    /// Build an octree from a slice of points
    pub fn build(points: &[P]) -> Self {
        if points.is_empty() {
            return Self::new(BoundingBox::new([0.0; 3], [1.0; 3]), 8, 10);
        }

        // Calculate bounding box of all points
        let mut min = points[0].position();
        let mut max = min;

        for point in points {
            let pos = point.position();
            for i in 0..3 {
                min[i] = min[i].min(pos[i]);
                max[i] = max[i].max(pos[i]);
            }
        }

        // Add some padding
        let padding = 0.01;
        for i in 0..3 {
            min[i] -= padding;
            max[i] += padding;
        }

        let bounds = BoundingBox::new(min, max);
        let mut octree = Self::new(bounds, 8, 10);

        // Insert all points
        for point in points {
            octree.insert(point.clone());
        }

        octree
    }

    /// Insert a point into the octree
    pub fn insert(&mut self, point: P) {
        if self.root.is_none() {
            self.root = Some(Box::new(OctreeNode::new(self.bounds, 0)));
        }

        if let Some(root) = &mut self.root {
            root.insert(point, self.max_depth, self.max_points_per_node);
        }
    }

    /// Find all points within a given radius of a query point
    pub fn radius_search(&self, query: &P, radius: f32) -> Vec<&P> {
        let mut results = Vec::new();

        if let Some(root) = &self.root {
            root.radius_search(query, radius, &mut results);
        }

        results
    }
}

impl<P: Point> OctreeNode<P> {
    /// Create a new octree node
    fn new(bounds: BoundingBox, depth: usize) -> Self {
        Self {
            bounds,
            points: Vec::new(),
            children: None,
            depth,
        }
    }

    /// Insert a point into this node
    fn insert(&mut self, point: P, max_depth: usize, max_points_per_node: usize) {
        // If this is a leaf node and we haven't exceeded limits, add the point
        if self.children.is_none() {
            self.points.push(point);

            // Check if we need to subdivide
            if self.points.len() > max_points_per_node && self.depth < max_depth {
                self.subdivide();

                // Redistribute points to children
                let points = std::mem::take(&mut self.points);
                for p in points {
                    self.insert_into_child(p);
                }
            }
        } else {
            // This is an internal node, insert into appropriate child
            self.insert_into_child(point);
        }
    }

    /// Subdivide this node into 8 children
    fn subdivide(&mut self) {
        let center = self.bounds.center();
        let min = self.bounds.min;
        let max = self.bounds.max;

        let children = [
            // Bottom children (z = min)
            Box::new(OctreeNode::new(
                BoundingBox::new([min[0], min[1], min[2]], [center[0], center[1], center[2]]),
                self.depth + 1,
            )),
            Box::new(OctreeNode::new(
                BoundingBox::new([center[0], min[1], min[2]], [max[0], center[1], center[2]]),
                self.depth + 1,
            )),
            Box::new(OctreeNode::new(
                BoundingBox::new([min[0], center[1], min[2]], [center[0], max[1], center[2]]),
                self.depth + 1,
            )),
            Box::new(OctreeNode::new(
                BoundingBox::new([center[0], center[1], min[2]], [max[0], max[1], center[2]]),
                self.depth + 1,
            )),
            // Top children (z = max)
            Box::new(OctreeNode::new(
                BoundingBox::new([min[0], min[1], center[2]], [center[0], center[1], max[2]]),
                self.depth + 1,
            )),
            Box::new(OctreeNode::new(
                BoundingBox::new([center[0], min[1], center[2]], [max[0], center[1], max[2]]),
                self.depth + 1,
            )),
            Box::new(OctreeNode::new(
                BoundingBox::new([min[0], center[1], center[2]], [center[0], max[1], max[2]]),
                self.depth + 1,
            )),
            Box::new(OctreeNode::new(
                BoundingBox::new([center[0], center[1], center[2]], [max[0], max[1], max[2]]),
                self.depth + 1,
            )),
        ];

        self.children = Some(children);
    }

    /// Insert a point into the appropriate child
    fn insert_into_child(&mut self, point: P) {
        if let Some(children) = &mut self.children {
            let pos = point.position();
            let center = self.bounds.center();

            let index = (if pos[0] >= center[0] { 1 } else { 0 })
                + (if pos[1] >= center[1] { 2 } else { 0 })
                + (if pos[2] >= center[2] { 4 } else { 0 });

            children[index].insert(point, 8, 10); // Use default values for simplicity
        }
    }

    /// Find all points within a given radius of the query point
    fn radius_search<'a>(&'a self, query: &P, radius: f32, results: &mut Vec<&'a P>) {
        let query_pos = query.position();
        let radius_squared = radius * radius;

        // Check if query sphere intersects with this node's bounds
        let mut min_dist_squared = 0.0;
        for i in 0..3 {
            if query_pos[i] < self.bounds.min[i] {
                let d = self.bounds.min[i] - query_pos[i];
                min_dist_squared += d * d;
            } else if query_pos[i] > self.bounds.max[i] {
                let d = query_pos[i] - self.bounds.max[i];
                min_dist_squared += d * d;
            }
        }

        if min_dist_squared > radius_squared {
            return; // No intersection possible
        }

        // Check points in this node
        for point in &self.points {
            let point_pos = point.position();
            let dist_squared = (0..3)
                .map(|i| {
                    let d = query_pos[i] - point_pos[i];
                    d * d
                })
                .sum::<f32>();

            if dist_squared <= radius_squared {
                results.push(point);
            }
        }

        // Recursively search children
        if let Some(children) = &self.children {
            for child in children.iter() {
                child.radius_search(query, radius, results);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::PointXYZ;

    #[test]
    fn test_bounding_box() {
        let bbox = BoundingBox::new([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        assert!(bbox.contains([0.5, 0.5, 0.5]));
        assert!(!bbox.contains([1.5, 0.5, 0.5]));

        let center = bbox.center();
        assert_eq!(center, [0.5, 0.5, 0.5]);
    }

    #[test]
    fn test_octree_build() {
        let points = vec![
            PointXYZ::new(0.0, 0.0, 0.0),
            PointXYZ::new(1.0, 1.0, 1.0),
            PointXYZ::new(2.0, 2.0, 2.0),
        ];

        let octree = Octree::build(&points);
        assert!(octree.root.is_some());
    }

    #[test]
    fn test_octree_radius_search() {
        let points = vec![
            PointXYZ::new(0.0, 0.0, 0.0),
            PointXYZ::new(0.1, 0.1, 0.1),
            PointXYZ::new(10.0, 10.0, 10.0),
        ];

        let octree = Octree::build(&points);
        let query = PointXYZ::new(0.0, 0.0, 0.0);

        let results = octree.radius_search(&query, 1.0);
        assert_eq!(results.len(), 2); // Should find first two points
    }
}
