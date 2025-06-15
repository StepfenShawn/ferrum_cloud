//! High-level point cloud viewer
//!
//! This module provides a high-level interface for visualizing point clouds,
//! combining the renderer, camera, and window management into a single easy-to-use API.

use crate::core::{Point, PointCloud};
use crate::error::{CloudError, Result};
use crate::visualization::{
    camera::{Camera, CameraController},
    config::{ColorScheme, RenderMode, ViewerConfig},
    renderer::PointCloudRenderer,
    window::{EventHandler, WindowManager, WindowManagerBuilder},
};
use std::collections::HashMap;
use std::sync::Arc;
use std::time::Instant;
use winit::{dpi::PhysicalSize, event::WindowEvent, window::Window};

/// Stored point cloud data before renderer initialization
struct PendingPointCloud {
    /// Serialized point data
    data: Vec<u8>,
    /// Point cloud name
    name: String,
    /// Point type information
    point_type: String,
}

/// High-level point cloud viewer
pub struct PointCloudViewer {
    /// Viewer configuration
    config: ViewerConfig,

    /// Window manager
    window_manager: Option<WindowManager>,

    /// Point cloud renderer
    renderer: Option<PointCloudRenderer>,

    /// Camera
    camera: Option<Camera>,

    /// Camera controller
    camera_controller: Option<CameraController>,

    /// Last frame time for delta time calculation
    last_frame_time: Instant,

    /// Point clouds waiting to be added to renderer
    pending_clouds: HashMap<String, Box<dyn std::any::Any + Send + Sync>>,
}

impl PointCloudViewer {
    /// Create a new point cloud viewer with default configuration
    pub async fn new() -> Result<Self> {
        Self::with_config(ViewerConfig::default()).await
    }

    /// Create a new point cloud viewer with custom configuration
    pub async fn with_config(config: ViewerConfig) -> Result<Self> {
        Ok(Self {
            config,
            window_manager: None,
            renderer: None,
            camera: None,
            camera_controller: None,
            last_frame_time: Instant::now(),
            pending_clouds: HashMap::new(),
        })
    }

    /// Add a point cloud to the viewer
    pub fn add_point_cloud<P: Point + Clone + Send + Sync + 'static>(
        &mut self,
        cloud: &PointCloud<P>,
        name: &str,
    ) -> Result<()> {
        if let Some(renderer) = &mut self.renderer {
            // Renderer is initialized, add directly
            renderer.add_point_cloud(cloud, name)?;
        } else {
            // Store for later when renderer is initialized
            self.pending_clouds.insert(name.to_string(), Box::new(cloud.clone()));
        }
        Ok(())
    }

    /// Remove a point cloud from the viewer
    pub fn remove_point_cloud(&mut self, name: &str) -> bool {
        // Remove from pending clouds first
        let was_pending = self.pending_clouds.remove(name).is_some();
        
        // Remove from renderer if it exists
        if let Some(renderer) = &mut self.renderer {
            renderer.remove_point_cloud(name) || was_pending
        } else {
            was_pending
        }
    }

    /// Set visibility of a point cloud
    pub fn set_point_cloud_visibility(&mut self, name: &str, visible: bool) -> bool {
        if let Some(renderer) = &mut self.renderer {
            renderer.set_point_cloud_visibility(name, visible)
        } else {
            // For pending clouds, we can't set visibility yet
            // This could be enhanced to store visibility state
            false
        }
    }

    /// Set the render mode
    pub fn set_render_mode(&mut self, mode: RenderMode) {
        if let Some(renderer) = &mut self.renderer {
            renderer.set_render_mode(mode);
        }
    }

    /// Set the color scheme
    pub fn set_color_scheme(&mut self, scheme: ColorScheme) {
        if let Some(renderer) = &mut self.renderer {
            renderer.set_color_scheme(scheme);
        }
    }

    /// Run the viewer (this will block until the window is closed)
    pub async fn run(self) -> Result<()> {
        // Create window manager
        let mut window_manager = WindowManagerBuilder::new()
            .title(&self.config.title)
            .size(self.config.width, self.config.height)
            .resizable(self.config.resizable)
            .vsync(self.config.vsync)
            .max_fps(self.config.max_fps)
            .camera_speed(self.config.camera_speed)
            .mouse_sensitivity(self.config.mouse_sensitivity)
            .zoom_speed(self.config.zoom_speed)
            .build();

        // Add viewer as event handler
        let viewer_handler = ViewerEventHandler::new(self);
        window_manager.add_event_handler(Box::new(viewer_handler));

        // Run the window event loop
        window_manager.run().await
    }

    /// Initialize the viewer components (called when window is created)
    async fn initialize(&mut self, window: Arc<Window>) -> Result<()> {
        let size = window.inner_size();

        // Create camera
        let aspect = size.width as f32 / size.height as f32;
        self.camera = Some(Camera::new(aspect));

        // Create camera controller
        self.camera_controller = Some(CameraController::new(
            self.config.camera_speed,
            self.config.mouse_sensitivity,
            self.config.zoom_speed,
        ));

        // Create renderer
        self.renderer = Some(PointCloudRenderer::new(window, self.config.render.clone()).await?);

        // Add all pending point clouds to the renderer
        self.add_pending_clouds_to_renderer()?;

        Ok(())
    }

    /// Add all pending point clouds to the renderer
    fn add_pending_clouds_to_renderer(&mut self) -> Result<()> {
        if let Some(renderer) = &mut self.renderer {
            let pending_clouds: Vec<(String, Box<dyn std::any::Any + Send + Sync>)> = 
                self.pending_clouds.drain().collect();
            
            for (name, cloud_any) in pending_clouds {
                // First try PointXYZRGB
                match cloud_any.downcast::<PointCloud<crate::core::PointXYZRGB>>() {
                    Ok(cloud) => {
                        renderer.add_point_cloud(&*cloud, &name)?;
                    }
                    Err(cloud_any) => {
                        // If that fails, try PointXYZ
                        match cloud_any.downcast::<PointCloud<crate::core::PointXYZ>>() {
                            Ok(cloud) => {
                                renderer.add_point_cloud(&*cloud, &name)?;
                            }
                            Err(_) => {
                                eprintln!("Warning: Could not add point cloud '{}' - unsupported point type", name);
                            }
                        }
                    }
                }
            }
        }
        Ok(())
    }

    /// Update the viewer (called each frame)
    fn update(&mut self) -> Result<()> {
        let now = Instant::now();
        let dt = (now - self.last_frame_time).as_secs_f32();
        self.last_frame_time = now;

        // Update camera
        if let (Some(camera), Some(controller)) = (&mut self.camera, &self.camera_controller) {
            controller.update_camera(camera, dt);
        }

        Ok(())
    }

    /// Render the viewer (called each frame)
    fn render(&mut self) -> Result<()> {
        if let (Some(renderer), Some(camera)) = (&mut self.renderer, &self.camera) {
            renderer.render(camera)?;
        }
        Ok(())
    }

    /// Handle window resize
    fn resize(&mut self, new_size: PhysicalSize<u32>) -> Result<()> {
        // Update camera aspect ratio
        if let Some(camera) = &mut self.camera {
            let aspect = new_size.width as f32 / new_size.height as f32;
            camera.set_aspect(aspect);
        }

        // Update renderer
        if let Some(renderer) = &mut self.renderer {
            renderer.resize(new_size)?;
        }

        Ok(())
    }
}

/// Event handler implementation for the point cloud viewer
struct ViewerEventHandler {
    viewer: PointCloudViewer,
    initialized: bool,
}

impl ViewerEventHandler {
    fn new(viewer: PointCloudViewer) -> Self {
        Self {
            viewer,
            initialized: false,
        }
    }
}

impl EventHandler for ViewerEventHandler {
    fn handle_event(&mut self, event: &WindowEvent, window: &Window) -> bool {
        // Pass events to camera controller if initialized
        if self.initialized {
            if let Some(controller) = &mut self.viewer.camera_controller {
                controller.process_event(event);
            }
        }

        true
    }

    fn handle_redraw(&mut self, _window: &Window) -> Result<()> {
        if self.initialized {
            self.viewer.update()?;
            self.viewer.render()?;
        }
        Ok(())
    }

    fn handle_resize(&mut self, new_size: PhysicalSize<u32>) -> Result<()> {
        if self.initialized {
            self.viewer.resize(new_size)?;
        }
        Ok(())
    }

    fn is_initialized(&self) -> bool {
        self.initialized
    }

    fn try_initialize(&mut self, window: Arc<Window>) -> Result<()> {
        if !self.initialized {
            // Use pollster to run async code in sync context
            pollster::block_on(self.viewer.initialize(window))?;
            self.initialized = true;
        }
        Ok(())
    }
}

/// Builder for creating point cloud viewers with custom configurations
pub struct PointCloudViewerBuilder {
    config: ViewerConfig,
}

impl PointCloudViewerBuilder {
    /// Create a new builder with default configuration
    pub fn new() -> Self {
        Self {
            config: ViewerConfig::default(),
        }
    }

    /// Set the window title
    pub fn title<S: Into<String>>(mut self, title: S) -> Self {
        self.config.title = title.into();
        self
    }

    /// Set the window size
    pub fn size(mut self, width: u32, height: u32) -> Self {
        self.config.width = width;
        self.config.height = height;
        self
    }

    /// Set whether the window is resizable
    pub fn resizable(mut self, resizable: bool) -> Self {
        self.config.resizable = resizable;
        self
    }

    /// Enable or disable VSync
    pub fn vsync(mut self, vsync: bool) -> Self {
        self.config.vsync = vsync;
        self
    }

    /// Set the maximum FPS
    pub fn max_fps(mut self, max_fps: u32) -> Self {
        self.config.max_fps = max_fps;
        self
    }

    /// Set the point size
    pub fn point_size(mut self, size: f32) -> Self {
        self.config.render.point_size = size;
        self
    }

    /// Set the background color
    pub fn background_color(mut self, color: [f32; 4]) -> Self {
        self.config.render.background_color = color;
        self
    }

    /// Set the default point color
    pub fn default_point_color(mut self, color: [f32; 3]) -> Self {
        self.config.render.default_point_color = color;
        self
    }

    /// Enable or disable depth testing
    pub fn depth_test(mut self, enabled: bool) -> Self {
        self.config.render.depth_test = enabled;
        self
    }

    /// Set the camera movement speed
    pub fn camera_speed(mut self, speed: f32) -> Self {
        self.config.camera_speed = speed;
        self
    }

    /// Set the mouse sensitivity
    pub fn mouse_sensitivity(mut self, sensitivity: f32) -> Self {
        self.config.mouse_sensitivity = sensitivity;
        self
    }

    /// Set the zoom speed
    pub fn zoom_speed(mut self, zoom_speed: f32) -> Self {
        self.config.zoom_speed = zoom_speed;
        self
    }

    /// Build the point cloud viewer
    pub async fn build(self) -> Result<PointCloudViewer> {
        PointCloudViewer::with_config(self.config).await
    }
}

impl Default for PointCloudViewerBuilder {
    fn default() -> Self {
        Self::new()
    }
}
