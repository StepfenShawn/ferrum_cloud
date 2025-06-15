//! WGPU-based point cloud renderer
//!
//! This module provides hardware-accelerated rendering of point clouds using
//! the WGPU graphics API. It supports various point types and rendering modes.

use crate::core::{Point, PointCloud, PointXYZRGB};
use crate::error::{CloudError, Result};
use crate::visualization::camera::Camera;
use crate::visualization::config::{ColorScheme, RenderConfig, RenderMode};
use std::collections::HashMap;
use std::sync::Arc;
use wgpu::util::DeviceExt;
use winit::window::Window;

/// Vertex data for point rendering
#[repr(C)]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
struct PointVertex {
    position: [f32; 3],
    color: [f32; 3],
}

impl PointVertex {
    /// Create vertex layout descriptor
    fn desc() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<PointVertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &[
                wgpu::VertexAttribute {
                    offset: 0,
                    shader_location: 0,
                    format: wgpu::VertexFormat::Float32x3,
                },
                wgpu::VertexAttribute {
                    offset: std::mem::size_of::<[f32; 3]>() as wgpu::BufferAddress,
                    shader_location: 1,
                    format: wgpu::VertexFormat::Float32x3,
                },
            ],
        }
    }
}

/// Uniform data for shaders
#[repr(C)]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
struct Uniforms {
    view_proj: [[f32; 4]; 4],
    point_size: f32,
    _padding: [f32; 3],
}

/// Point cloud data stored on GPU
struct PointCloudData {
    vertex_buffer: wgpu::Buffer,
    vertex_count: u32,
    name: String,
    visible: bool,
}

/// WGPU-based point cloud renderer
pub struct PointCloudRenderer {
    /// WGPU device
    device: wgpu::Device,

    /// WGPU command queue
    queue: wgpu::Queue,

    /// Surface for rendering
    surface: wgpu::Surface<'static>,

    /// Surface configuration
    config: wgpu::SurfaceConfiguration,

    /// Render pipeline
    render_pipeline: wgpu::RenderPipeline,

    /// Uniform buffer
    uniform_buffer: wgpu::Buffer,

    /// Bind group for uniforms
    uniform_bind_group: wgpu::BindGroup,

    /// Stored point clouds
    point_clouds: HashMap<String, PointCloudData>,

    /// Render configuration
    render_config: RenderConfig,

    /// Current render mode
    render_mode: RenderMode,

    /// Current color scheme
    color_scheme: ColorScheme,
}

impl PointCloudRenderer {
    /// Create a new point cloud renderer
    pub async fn new(window: Arc<Window>, render_config: RenderConfig) -> Result<Self> {
        let size = window.inner_size();

        // Create WGPU instance
        let instance = wgpu::Instance::new(&wgpu::InstanceDescriptor {
            backends: wgpu::Backends::all(),
            ..Default::default()
        });

        // Create surface
        let surface = instance
            .create_surface(window.clone())
            .map_err(|e| CloudError::Visualization(format!("Failed to create surface: {}", e)))?;

        // Request adapter
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::default(),
                compatible_surface: Some(&surface),
                force_fallback_adapter: false,
            })
            .await
            .or_else(|_| {
                Err(CloudError::Visualization(
                    "Failed to find suitable adapter".to_string(),
                ))
            })?;

        // Request device and queue
        let (device, queue) = adapter
            .request_device(
                &wgpu::DeviceDescriptor {
                    label: None,
                    required_features: wgpu::Features::empty(),
                    required_limits: wgpu::Limits::default(),
                    memory_hints: wgpu::MemoryHints::default(),
                    trace: wgpu::Trace::default(),
                },
                // None,
            )
            .await
            .map_err(|e| CloudError::Visualization(format!("Failed to create device: {}", e)))?;

        // Configure surface
        let surface_caps = surface.get_capabilities(&adapter);
        let surface_format = surface_caps
            .formats
            .iter()
            .copied()
            .find(|f| f.is_srgb())
            .unwrap_or(surface_caps.formats[0]);

        let config = wgpu::SurfaceConfiguration {
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            format: surface_format,
            width: size.width,
            height: size.height,
            present_mode: if render_config.depth_test {
                wgpu::PresentMode::Fifo
            } else {
                wgpu::PresentMode::Immediate
            },
            alpha_mode: surface_caps.alpha_modes[0],
            view_formats: vec![],
            desired_maximum_frame_latency: 2,
        };
        surface.configure(&device, &config);

        // Create shaders
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Point Cloud Shader"),
            source: wgpu::ShaderSource::Wgsl(POINT_CLOUD_SHADER.into()),
        });

        // Create uniform buffer
        let uniform_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Uniform Buffer"),
            size: std::mem::size_of::<Uniforms>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        // Create bind group layout
        let uniform_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                }],
                label: Some("uniform_bind_group_layout"),
            });

        // Create bind group
        let uniform_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            layout: &uniform_bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform_buffer.as_entire_binding(),
            }],
            label: Some("uniform_bind_group"),
        });

        // Create render pipeline layout
        let render_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("Render Pipeline Layout"),
                bind_group_layouts: &[&uniform_bind_group_layout],
                push_constant_ranges: &[],
            });

        // Create render pipeline
        let render_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Point Cloud Render Pipeline"),
            layout: Some(&render_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_main"),
                buffers: &[PointVertex::desc()],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_main"),
                targets: &[Some(wgpu::ColorTargetState {
                    format: config.format,
                    blend: Some(wgpu::BlendState::REPLACE),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::PointList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: None,
                polygon_mode: wgpu::PolygonMode::Fill,
                unclipped_depth: false,
                conservative: false,
            },
            depth_stencil: if render_config.depth_test {
                Some(wgpu::DepthStencilState {
                    format: wgpu::TextureFormat::Depth32Float,
                    depth_write_enabled: true,
                    depth_compare: wgpu::CompareFunction::Less,
                    stencil: wgpu::StencilState::default(),
                    bias: wgpu::DepthBiasState::default(),
                })
            } else {
                None
            },
            multisample: wgpu::MultisampleState {
                count: 1,
                mask: !0,
                alpha_to_coverage_enabled: false,
            },
            multiview: None,
            cache: None,
        });

        Ok(Self {
            device,
            queue,
            surface,
            config,
            render_pipeline,
            uniform_buffer,
            uniform_bind_group,
            point_clouds: HashMap::new(),
            render_config,
            render_mode: RenderMode::default(),
            color_scheme: ColorScheme::default(),
        })
    }

    /// Add a point cloud to the renderer
    pub fn add_point_cloud<P: Point + 'static>(
        &mut self,
        cloud: &PointCloud<P>,
        name: &str,
    ) -> Result<()> {
        let vertices = self.create_vertices(cloud)?;

        let vertex_buffer = self
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some(&format!("{} Vertex Buffer", name)),
                contents: bytemuck::cast_slice(&vertices),
                usage: wgpu::BufferUsages::VERTEX,
            });

        let point_cloud_data = PointCloudData {
            vertex_buffer,
            vertex_count: vertices.len() as u32,
            name: name.to_string(),
            visible: true,
        };

        self.point_clouds.insert(name.to_string(), point_cloud_data);
        Ok(())
    }

    /// Remove a point cloud from the renderer
    pub fn remove_point_cloud(&mut self, name: &str) -> bool {
        self.point_clouds.remove(name).is_some()
    }

    /// Set visibility of a point cloud
    pub fn set_point_cloud_visibility(&mut self, name: &str, visible: bool) -> bool {
        if let Some(cloud) = self.point_clouds.get_mut(name) {
            cloud.visible = visible;
            true
        } else {
            false
        }
    }

    /// Set the render mode
    pub fn set_render_mode(&mut self, mode: RenderMode) {
        self.render_mode = mode;
    }

    /// Set the color scheme
    pub fn set_color_scheme(&mut self, scheme: ColorScheme) {
        self.color_scheme = scheme;
    }

    /// Resize the renderer
    pub fn resize(&mut self, new_size: winit::dpi::PhysicalSize<u32>) -> Result<()> {
        if new_size.width > 0 && new_size.height > 0 {
            self.config.width = new_size.width;
            self.config.height = new_size.height;
            self.surface.configure(&self.device, &self.config);
        }
        Ok(())
    }

    /// Render the point clouds
    pub fn render(&mut self, camera: &Camera) -> Result<()> {
        // Update uniforms
        let uniforms = Uniforms {
            view_proj: camera.view_projection_matrix(),
            point_size: self.render_config.point_size,
            _padding: [0.0; 3],
        };

        self.queue
            .write_buffer(&self.uniform_buffer, 0, bytemuck::cast_slice(&[uniforms]));

        // Get surface texture
        let output = self.surface.get_current_texture().map_err(|e| {
            CloudError::Visualization(format!("Failed to get surface texture: {}", e))
        })?;

        let view = output
            .texture
            .create_view(&wgpu::TextureViewDescriptor::default());

        // Create command encoder
        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("Render Encoder"),
            });

        // Begin render pass
        {
            let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("Point Cloud Render Pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: self.render_config.background_color[0] as f64,
                            g: self.render_config.background_color[1] as f64,
                            b: self.render_config.background_color[2] as f64,
                            a: self.render_config.background_color[3] as f64,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                occlusion_query_set: None,
                timestamp_writes: None,
            });

            render_pass.set_pipeline(&self.render_pipeline);
            render_pass.set_bind_group(0, &self.uniform_bind_group, &[]);

            // Render all visible point clouds
            for cloud in self.point_clouds.values() {
                if cloud.visible {
                    render_pass.set_vertex_buffer(0, cloud.vertex_buffer.slice(..));
                    render_pass.draw(0..cloud.vertex_count, 0..1);
                }
            }
        }

        // Submit commands
        self.queue.submit(std::iter::once(encoder.finish()));
        output.present();

        Ok(())
    }

    /// Create vertices from a point cloud
    fn create_vertices<P: Point + 'static>(
        &self,
        cloud: &PointCloud<P>,
    ) -> Result<Vec<PointVertex>> {
        let mut vertices = Vec::with_capacity(cloud.len());

        for point in cloud.points() {
            let position = point.position();
            let color = self.get_point_color(point);

            vertices.push(PointVertex { position, color });
        }

        Ok(vertices)
    }

    /// Get color for a point based on the current color scheme
    fn get_point_color<P: Point + 'static>(&self, point: &P) -> [f32; 3] {
        match self.color_scheme {
            ColorScheme::Original => {
                // Try to get RGB color if available
                if let Some(rgb_point) = (point as &dyn std::any::Any).downcast_ref::<PointXYZRGB>()
                {
                    rgb_point.rgb_normalized()
                } else {
                    self.render_config.default_point_color
                }
            }
            ColorScheme::Height => {
                let z = point.z();
                let normalized = (z + 10.0) / 20.0; // Assume height range -10 to 10
                [normalized, 1.0 - normalized, 0.5]
            }
            ColorScheme::Distance => {
                let pos = point.position();
                let distance = (pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2]).sqrt();
                let normalized = (distance / 10.0).min(1.0);
                [normalized, 0.5, 1.0 - normalized]
            }
            ColorScheme::Normal => {
                // This would require normal information
                self.render_config.default_point_color
            }
            ColorScheme::Uniform(color) => color,
        }
    }
}

/// WGSL shader source for point cloud rendering
const POINT_CLOUD_SHADER: &str = r#"
struct Uniforms {
    view_proj: mat4x4<f32>,
    point_size: f32,
}

@group(0) @binding(0)
var<uniform> uniforms: Uniforms;

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) color: vec3<f32>,
}

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) color: vec3<f32>,
}

@vertex
fn vs_main(input: VertexInput) -> VertexOutput {
    var out: VertexOutput;
    out.clip_position = uniforms.view_proj * vec4<f32>(input.position, 1.0);
    out.color = input.color;
    return out;
}

@fragment
fn fs_main(input: VertexOutput) -> @location(0) vec4<f32> {
    return vec4<f32>(input.color, 1.0);
}
"#;
