//! Point cloud visualization module
//!
//! This module provides real-time 3D visualization capabilities for point clouds
//! using modern graphics APIs. It supports interactive viewing, camera controls,
//! and various rendering modes.
//!
//! ## Features
//!
//! - **Real-time Rendering**: Hardware-accelerated point cloud rendering using WGPU
//! - **Interactive Controls**: Mouse and keyboard controls for camera navigation
//! - **Multiple Point Types**: Support for XYZ, XYZRGB, and normal visualization
//! - **Flexible Rendering**: Configurable point sizes, colors, and rendering modes
//! - **Cross-platform**: Works on Windows, macOS, and Linux
//!
//! ## Quick Start
//!
//! ```rust
//! use ferrum_cloud::prelude::*;
//! use ferrum_cloud::visualization::*;
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

pub mod camera;
pub mod config;
pub mod renderer;
pub mod viewer;
pub mod window;

// Re-export commonly used types
pub use camera::{Camera, CameraController};
pub use config::{RenderConfig, ViewerConfig};
pub use renderer::PointCloudRenderer;
pub use viewer::PointCloudViewer;
pub use window::WindowManager;

use crate::error::{CloudError, Result};
