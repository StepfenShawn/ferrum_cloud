//! Spatial search structures and algorithms
//!
//! This module provides efficient spatial search structures for point clouds,
//! including KD-trees and octrees for nearest neighbor search.

pub mod kdtree;
pub mod octree;

// Re-export commonly used types
pub use kdtree::KdTree;
pub use octree::Octree;
