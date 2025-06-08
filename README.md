# ferrum_cloud

Pure Rust implementation of Point Cloud Library (PCL).

This library provides efficient, safe, and ergonomic APIs for point cloud processing,
leveraging Rust's ownership system, zero-cost abstractions, and parallel processing capabilities.

# Features

- **Memory Safety**: Leverages Rust's ownership system for safe memory management
- **Zero-Copy Operations**: Efficient processing through views and references
- **Parallel Processing**: Built-in support for parallel operations using Rayon
- **Generic Point Types**: Flexible point type system supporting various point formats
- **Comprehensive I/O**: Support for multiple point cloud file formats

# Quick Start

```rust
use ferrum_cloud::prelude::*;

fn main() -> Result<()> {
    // Load a point cloud
    let cloud = io::load_pcd("scene.pcd")?;

    // Process the cloud
    let processed = cloud
        .voxel_downsample(0.05)
        .remove_outliers(50, 1.0)
        .estimate_normals(0.5);

    // Save the result
    io::save_ply(&processed, "processed.ply")?;

    Ok(())
}
```