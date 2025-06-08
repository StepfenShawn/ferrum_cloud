# ferrum_cloud
Pure Rust implementation of Point Cloud Library (PCL).

# Quick Start
```rust
use ferrumcloud::prelude::*;

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