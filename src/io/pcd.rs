//! PCD (Point Cloud Data) file format support
//!
//! This module provides functionality for reading and writing PCD files,
//! which is the native format of the Point Cloud Library (PCL).

use crate::core::{Metadata, Point, PointCloud, PointXYZ, PointXYZRGB};
use crate::error::{CloudError, Result};
use std::fs::File;
use std::io::{BufRead, BufReader, BufWriter, Write};
use std::path::Path;

/// Load a point cloud from a PCD file
///
/// # Arguments
/// * `path` - Path to the PCD file
///
/// # Returns
/// A Result containing the loaded PointCloud or an error
pub fn load_pcd<P: AsRef<Path>>(path: P) -> Result<PointCloud<PointXYZ>> {
    let file = File::open(path)?;
    let reader = BufReader::new(file);

    let mut points = Vec::new();
    let mut metadata = Metadata::default();
    let mut in_data_section = false;

    for line in reader.lines() {
        let line = line?;
        let line = line.trim();

        if line.is_empty() || line.starts_with('#') {
            continue;
        }

        if line.starts_with("VERSION") {
            // Store version in custom fields
            if let Some(version) = line.split_whitespace().nth(1) {
                metadata
                    .custom_fields
                    .insert("version".to_string(), version.to_string());
            }
        } else if line.starts_with("FIELDS") {
            // Store field information in custom fields
            let fields: Vec<&str> = line.split_whitespace().skip(1).collect();
            let fields_str = fields.join(",");
            metadata
                .custom_fields
                .insert("fields".to_string(), fields_str);
        } else if line.starts_with("SIZE") {
            // Parse size information
        } else if line.starts_with("TYPE") {
            // Parse type information
        } else if line.starts_with("COUNT") {
            // Parse count information
        } else if line.starts_with("WIDTH") {
            metadata.width = line
                .split_whitespace()
                .nth(1)
                .and_then(|s| s.parse().ok())
                .unwrap_or(0);
        } else if line.starts_with("HEIGHT") {
            metadata.height = line
                .split_whitespace()
                .nth(1)
                .and_then(|s| s.parse().ok())
                .unwrap_or(1);
        } else if line.starts_with("VIEWPOINT") {
            // Parse viewpoint information
        } else if line.starts_with("POINTS") {
            let point_count = line
                .split_whitespace()
                .nth(1)
                .and_then(|s| s.parse().ok())
                .unwrap_or(0);
            points.reserve(point_count);
        } else if line.starts_with("DATA") {
            in_data_section = true;
        } else if in_data_section {
            // Parse point data
            let coords: Vec<&str> = line.split_whitespace().collect();
            if coords.len() >= 3 {
                if let (Ok(x), Ok(y), Ok(z)) = (
                    coords[0].parse::<f32>(),
                    coords[1].parse::<f32>(),
                    coords[2].parse::<f32>(),
                ) {
                    points.push(PointXYZ::new(x, y, z));
                }
            }
        }
    }

    // Update metadata with actual point count
    metadata.width = points.len() as u32;
    metadata.height = 1;
    metadata.is_organized = false;

    Ok(PointCloud::from_points_and_metadata(points, metadata))
}

/// Save a point cloud to a PCD file
///
/// # Arguments
/// * `cloud` - The point cloud to save
/// * `path` - Path where to save the PCD file
///
/// # Returns
/// A Result indicating success or failure
pub fn save_pcd<T: Point, P: AsRef<Path>>(cloud: &PointCloud<T>, path: P) -> Result<()> {
    let file = File::create(path)?;
    let mut writer = BufWriter::new(file);

    // Write header
    writeln!(writer, "# .PCD v0.7 - Point Cloud Data file format")?;
    writeln!(writer, "VERSION 0.7")?;
    writeln!(writer, "FIELDS x y z")?;
    writeln!(writer, "SIZE 4 4 4")?;
    writeln!(writer, "TYPE F F F")?;
    writeln!(writer, "COUNT 1 1 1")?;
    writeln!(writer, "WIDTH {}", cloud.len())?;
    writeln!(writer, "HEIGHT 1")?;
    writeln!(writer, "VIEWPOINT 0 0 0 1 0 0 0")?;
    writeln!(writer, "POINTS {}", cloud.len())?;
    writeln!(writer, "DATA ascii")?;

    // Write point data
    for point in cloud.points() {
        let pos = point.position();
        writeln!(writer, "{} {} {}", pos[0], pos[1], pos[2])?;
    }

    writer.flush()?;
    Ok(())
}

#[derive(Debug, Clone, Copy)]
enum DataFormat {
    Ascii,
    Binary,
    BinaryCompressed,
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::NamedTempFile;

    #[test]
    fn test_pcd_roundtrip() {
        let points = vec![
            PointXYZ::new(1.0, 2.0, 3.0),
            PointXYZ::new(4.0, 5.0, 6.0),
            PointXYZ::new(7.0, 8.0, 9.0),
        ];
        let original_cloud = PointCloud::from_points(points);

        let temp_file = NamedTempFile::new().unwrap();
        let temp_path = temp_file.path();

        // Save and load
        save_pcd(&original_cloud, temp_path).unwrap();
        let loaded_cloud = load_pcd(temp_path).unwrap();

        assert_eq!(original_cloud.len(), loaded_cloud.len());

        for (original, loaded) in original_cloud
            .points()
            .iter()
            .zip(loaded_cloud.points().iter())
        {
            let orig_pos = original.position();
            let load_pos = loaded.position();
            assert!((orig_pos[0] - load_pos[0]).abs() < 1e-6);
            assert!((orig_pos[1] - load_pos[1]).abs() < 1e-6);
            assert!((orig_pos[2] - load_pos[2]).abs() < 1e-6);
        }
    }
}
