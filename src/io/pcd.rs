//! PCD (Point Cloud Data) file format support
//!
//! This module provides functionality for reading and writing PCD files,
//! which is the native format of the Point Cloud Library (PCL).
//!
//! Uses the `pcd-rs` crate for efficient and robust PCD file handling.

use crate::core::{Metadata, Point, PointCloud, PointXYZ};
use crate::error::{CloudError, Result};
use pcd_rs::{DataKind, DynReader, DynRecord, DynWriter, Field, Schema, ValueKind, WriterInit};
use std::path::Path;

/// Load a point cloud from a PCD file
///
/// # Arguments
/// * `path` - Path to the PCD file
///
/// # Returns
/// A Result containing the loaded PointCloud or an error
pub fn load_pcd<P: AsRef<Path>>(path: P) -> Result<PointCloud<PointXYZ>> {
    let reader = DynReader::open(path.as_ref())
        .map_err(|e| CloudError::format_error(format!("Failed to open PCD file: {}", e)))?;

    let mut points = Vec::new();
    let mut metadata = Metadata::default();

    // Extract metadata from PCD header
    let pcd_meta = reader.meta();
    metadata.width = pcd_meta.width as u32;
    metadata.height = pcd_meta.height as u32;
    metadata.is_organized = metadata.height > 1;

    // Store additional metadata
    metadata
        .custom_fields
        .insert("version".to_string(), "0.7".to_string());

    // Read point data
    for record_result in reader {
        let record = record_result
            .map_err(|e| CloudError::format_error(format!("Failed to read PCD record: {}", e)))?;

        // Extract x, y, z coordinates from the record
        let (x, y, z) = extract_xyz_from_record(&record)?;
        points.push(PointXYZ::new(x, y, z));
    }

    // Update metadata with actual point count
    metadata.width = points.len() as u32;
    if metadata.height == 0 {
        metadata.height = 1;
    }

    Ok(PointCloud::from_points_and_metadata(points, metadata))
}

/// Extract x, y, z coordinates from a DynRecord
fn extract_xyz_from_record(record: &DynRecord) -> Result<(f32, f32, f32)> {
    let fields = &record.0;

    if fields.len() < 3 {
        return Err(CloudError::format_error(
            "PCD record must have at least 3 fields (x, y, z)",
        ));
    }

    let x = extract_f32_from_field(&fields[0])?;
    let y = extract_f32_from_field(&fields[1])?;
    let z = extract_f32_from_field(&fields[2])?;

    Ok((x, y, z))
}

/// Extract f32 value from a Field
fn extract_f32_from_field(field: &Field) -> Result<f32> {
    match field {
        Field::F32(values) => {
            if values.is_empty() {
                Err(CloudError::format_error("Empty F32 field"))
            } else {
                Ok(values[0])
            }
        }
        Field::F64(values) => {
            if values.is_empty() {
                Err(CloudError::format_error("Empty F64 field"))
            } else {
                Ok(values[0] as f32)
            }
        }
        Field::I32(values) => {
            if values.is_empty() {
                Err(CloudError::format_error("Empty I32 field"))
            } else {
                Ok(values[0] as f32)
            }
        }
        _ => Err(CloudError::format_error(
            "Unsupported field type for coordinate",
        )),
    }
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
    // Define the schema for x, y, z coordinates
    let schema = vec![
        ("x", ValueKind::F32, 1),
        ("y", ValueKind::F32, 1),
        ("z", ValueKind::F32, 1),
    ];

    // Create writer with ASCII format
    let mut writer: DynWriter<_> = WriterInit {
        width: cloud.len() as u64,
        height: 1,
        viewpoint: Default::default(),
        data_kind: DataKind::Ascii,
        schema: Some(Schema::from_iter(schema)),
    }
    .create(path.as_ref())
    .map_err(|e| CloudError::format_error(format!("Failed to create PCD writer: {}", e)))?;

    // Write point data
    for point in cloud.points() {
        let pos = point.position();
        let record = DynRecord(vec![
            Field::F32(vec![pos[0]]),
            Field::F32(vec![pos[1]]),
            Field::F32(vec![pos[2]]),
        ]);

        writer
            .push(&record)
            .map_err(|e| CloudError::format_error(format!("Failed to write PCD record: {}", e)))?;
    }

    // Finalize the writer
    writer
        .finish()
        .map_err(|e| CloudError::format_error(format!("Failed to finalize PCD file: {}", e)))?;

    Ok(())
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
