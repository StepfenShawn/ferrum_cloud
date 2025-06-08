//! Core data structures and traits for point cloud processing
//!
//! This module contains the fundamental building blocks of the FerrumCloud library,
//! including point types, point cloud containers, and views.

pub mod cloud;
pub mod metadata;
pub mod point;
pub mod view;

// Re-export commonly used types
pub use cloud::PointCloud;
pub use metadata::Metadata;
pub use point::{Point, PointXYZ, PointXYZRGB, PointXYZRGBNormal};
pub use view::PointCloudView;
