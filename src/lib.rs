//! # ferrum_cloud
//!
//! Pure Rust implementation of Point Cloud Library (PCL).
//!
//! This library provides efficient, safe, and ergonomic APIs for point cloud processing,
//! leveraging Rust's ownership system, zero-cost abstractions, and parallel processing capabilities.
//!
//! ## Features
//!
//! - **Memory Safety**: Leverages Rust's ownership system for safe memory management
//! - **Zero-Copy Operations**: Efficient processing through views and references
//! - **Parallel Processing**: Built-in support for parallel operations using Rayon
//! - **Generic Point Types**: Flexible point type system supporting various point formats
//! - **Comprehensive I/O**: Support for multiple point cloud file formats
//! - **Real-time Visualization**: Hardware-accelerated 3D visualization (with `visualization` feature)
//!
//! ## Quick Start
//!
//! ```rust
//! use ferrum_cloud::prelude::*;
//!
//! fn main() -> Result<()> {
//!     // Load a point cloud
//!     let cloud = io::load_pcd("examples/scene.pcd")?;
//!
//!     // Process the cloud
//!     let processed = cloud
//!         .voxel_downsample(0.05)
//!         .remove_outliers(50, 1.0)?;
//!         //.estimate_normals(0.5)?;
//!
//!     // Save the result
//!     io::save_ply(&processed, "examples/processed.ply")?;
//!
//!     Ok(())
//! }
//! ```
//!
//! ## Visualization
//!
//! With the `visualization` feature enabled:
//!
//! ```rust,no_run
//! use ferrum_cloud::prelude::*;
//!
//! #[tokio::main]
//! async fn main() -> Result<()> {
//!     let cloud = io::load_pcd("examples/scene.pcd")?;
//!     
//!     let mut viewer = PointCloudViewer::new().await?;
//!     viewer.add_point_cloud(&cloud, "scene")?;
//!     viewer.run().await?;
//!     
//!     Ok(())
//! }
//! ```

pub mod algorithms;
pub mod core;
pub mod error;
pub mod io;
pub mod search;
pub mod utils;

#[cfg(feature = "visualization")]
pub mod visualization;

/// Prelude module for convenient imports
pub mod prelude {
    pub use crate::algorithms::*;
    pub use crate::core::{Point, PointCloud, PointCloudView, PointXYZ, PointXYZRGB};
    pub use crate::error::{CloudError, Result};
    pub use crate::io;
    pub use crate::search::*;

    #[cfg(feature = "visualization")]
    pub use crate::visualization::*;
}

// Re-export commonly used types
pub use crate::core::{Point, PointCloud, PointXYZ, PointXYZRGB};
pub use crate::error::{CloudError, Result};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::prelude::*;

    #[test]
    fn test_basic_point_creation() {
        let point = PointXYZ::new(1.0, 2.0, 3.0);
        assert_eq!(point.position(), [1.0, 2.0, 3.0]);
    }

    #[test]
    fn test_point_cloud_creation() {
        let points = vec![PointXYZ::new(0.0, 0.0, 0.0), PointXYZ::new(1.0, 1.0, 1.0)];
        let cloud = PointCloud::from_points(points);
        assert_eq!(cloud.len(), 2);
    }
}
