//! PLY (Polygon File Format) support
//!
//! This module provides functionality for reading and writing PLY files,
//! a popular format for storing 3D polygon data.

use crate::core::{Point, PointCloud, PointXYZ};
use crate::error::{CloudError, Result};
use std::fs::File;
use std::io::{BufRead, BufReader, BufWriter, Write};
use std::path::Path;

/// Load a point cloud from a PLY file
pub fn load_ply<P: AsRef<Path>>(path: P) -> Result<PointCloud<PointXYZ>> {
    let file = File::open(path)?;
    let reader = BufReader::new(file);
    parse_ply(reader)
}

/// Save a point cloud to a PLY file
pub fn save_ply<P: Point, Q: AsRef<Path>>(cloud: &PointCloud<P>, path: Q) -> Result<()> {
    let file = File::create(path)?;
    let writer = BufWriter::new(file);
    write_ply(cloud, writer)
}

/// Parse PLY data from a reader
fn parse_ply<R: BufRead>(reader: R) -> Result<PointCloud<PointXYZ>> {
    let mut lines = reader.lines();
    let mut points = Vec::new();
    let mut vertex_count = 0usize;
    let mut in_header = true;

    // Parse header
    while let Some(line) = lines.next() {
        let line = line?;
        let line = line.trim();

        if line.starts_with("ply") {
            continue;
        } else if line.starts_with("format") {
            // Check format - we only support ASCII for now
            if !line.contains("ascii") {
                return Err(CloudError::format_error(
                    "Only ASCII PLY format is supported",
                ));
            }
        } else if line.starts_with("element vertex") {
            let parts: Vec<&str> = line.split_whitespace().collect();
            if parts.len() >= 3 {
                vertex_count = parts[2]
                    .parse()
                    .map_err(|_| CloudError::format_error("Invalid vertex count"))?;
            }
        } else if line.starts_with("property") {
            // Property definitions - we expect x, y, z
            continue;
        } else if line.starts_with("end_header") {
            in_header = false;
            break;
        }
    }

    if in_header {
        return Err(CloudError::format_error("No end_header found"));
    }

    // Parse vertex data
    points.reserve(vertex_count);
    for line in lines.take(vertex_count) {
        let line = line?;
        let line = line.trim();

        if line.is_empty() {
            continue;
        }

        let coords: Vec<&str> = line.split_whitespace().collect();
        if coords.len() >= 3 {
            let x: f32 = coords[0]
                .parse()
                .map_err(|_| CloudError::format_error("Invalid x coordinate"))?;
            let y: f32 = coords[1]
                .parse()
                .map_err(|_| CloudError::format_error("Invalid y coordinate"))?;
            let z: f32 = coords[2]
                .parse()
                .map_err(|_| CloudError::format_error("Invalid z coordinate"))?;

            points.push(PointXYZ::new(x, y, z));
        }
    }

    Ok(PointCloud::from_points(points))
}

/// Write PLY data to a writer
fn write_ply<P: Point, W: Write>(cloud: &PointCloud<P>, mut writer: W) -> Result<()> {
    // Write header
    writeln!(writer, "ply")?;
    writeln!(writer, "format ascii 1.0")?;
    writeln!(writer, "element vertex {}", cloud.len())?;
    writeln!(writer, "property float x")?;
    writeln!(writer, "property float y")?;
    writeln!(writer, "property float z")?;
    writeln!(writer, "end_header")?;

    // Write vertex data
    for point in cloud.iter() {
        let pos = point.position();
        writeln!(writer, "{} {} {}", pos[0], pos[1], pos[2])?;
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    #[test]
    fn test_ply_parsing() {
        let ply_data = r#"ply
format ascii 1.0
element vertex 3
property float x
property float y
property float z
end_header
0.0 0.0 0.0
1.0 1.0 1.0
2.0 2.0 2.0
"#;

        let cursor = Cursor::new(ply_data);
        let cloud = parse_ply(cursor).unwrap();

        assert_eq!(cloud.len(), 3);
        assert_eq!(cloud.get(0).unwrap().position(), [0.0, 0.0, 0.0]);
        assert_eq!(cloud.get(1).unwrap().position(), [1.0, 1.0, 1.0]);
        assert_eq!(cloud.get(2).unwrap().position(), [2.0, 2.0, 2.0]);
    }

    #[test]
    fn test_ply_writing() {
        let points = vec![PointXYZ::new(0.0, 0.0, 0.0), PointXYZ::new(1.0, 1.0, 1.0)];
        let cloud = PointCloud::from_points(points);

        let mut buffer = Vec::new();
        write_ply(&cloud, &mut buffer).unwrap();

        let output = String::from_utf8(buffer).unwrap();
        assert!(output.contains("element vertex 2"));
        assert!(output.contains("0 0 0"));
        assert!(output.contains("1 1 1"));
    }
}
