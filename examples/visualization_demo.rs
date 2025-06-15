//! Point cloud visualization demo
//!
//! This example demonstrates how to use the visualization module to display
//! point clouds in real-time 3D.
//!
//! Usage:
//! ```bash
//! cargo run --example visualization_demo --features visualization
//! ```

#[cfg(feature = "visualization")]
use ferrum_cloud::prelude::*;
#[cfg(feature = "visualization")]
use ferrum_cloud::visualization::viewer::PointCloudViewerBuilder;

#[cfg(feature = "visualization")]
#[tokio::main]
async fn main() -> Result<()> {
    // Create a simple point cloud for demonstration
    let mut points = Vec::new();

    // Create a cube of points
    for x in -10..=10 {
        for y in -10..=10 {
            for z in -10..=10 {
                if (x as i32).abs() == 10 || (y as i32).abs() == 10 || (z as i32).abs() == 10 {
                    let point = PointXYZRGB::new(
                        x as f32 * 0.1,
                        y as f32 * 0.1,
                        z as f32 * 0.1,
                        ((x + 10) * 12) as u8,
                        ((y + 10) * 12) as u8,
                        ((z + 10) * 12) as u8,
                    );
                    points.push(point);
                }
            }
        }
    }

    // Create a sphere of points
    let mut sphere_points = Vec::new();
    for i in 0..1000 {
        let theta = (i as f32 / 1000.0) * 2.0 * std::f32::consts::PI;
        let phi = ((i * 7) % 1000) as f32 / 1000.0 * std::f32::consts::PI;

        let radius = 1.5;
        let x = radius * phi.sin() * theta.cos();
        let y = radius * phi.sin() * theta.sin();
        let z = radius * phi.cos();

        let point = PointXYZRGB::new(
            x + 3.0, // Offset to separate from cube
            y,
            z,
            255,
            (theta * 40.0) as u8,
            (phi * 80.0) as u8,
        );
        sphere_points.push(point);
    }

    // Create point clouds
    let cube_cloud = PointCloud::from_points(points);
    let sphere_cloud = PointCloud::from_points(sphere_points);

    println!("Created cube with {} points", cube_cloud.len());
    println!("Created sphere with {} points", sphere_cloud.len());

    // Create viewer with custom configuration
    let mut viewer = PointCloudViewerBuilder::new()
        .title("FerrumCloud Visualization Demo")
        .size(1280, 720)
        .point_size(3.0)
        .background_color([0.05, 0.05, 0.1, 1.0]) // Dark blue background
        .camera_speed(2.0)
        .mouse_sensitivity(0.003)
        .build()
        .await?;

    // Add point clouds to viewer
    viewer.add_point_cloud(&cube_cloud, "cube")?;
    viewer.add_point_cloud(&sphere_cloud, "sphere")?;

    println!("Starting visualization...");
    println!("Controls:");
    println!("  WASD - Move camera");
    println!("  Space/Shift - Move up/down");
    println!("  Mouse - Look around (hold left button)");
    println!("  Mouse wheel - Zoom");
    println!("  Close window to exit");

    // Run the viewer (this will block until window is closed)
    viewer.run().await?;

    println!("Visualization ended.");
    Ok(())
}

#[cfg(not(feature = "visualization"))]
fn main() {
    println!("This example requires the 'visualization' feature to be enabled.");
    println!("Run with: cargo run --example visualization_demo --features visualization");
}
