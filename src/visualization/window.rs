//! Window management for point cloud visualization
//!
//! This module handles window creation, event processing, and the main event loop
//! for the point cloud viewer application.

use crate::error::{CloudError, Result};
use crate::visualization::config::ViewerConfig;
use std::sync::Arc;
use winit::{
    application::ApplicationHandler,
    event::WindowEvent,
    event_loop::{ActiveEventLoop, ControlFlow, EventLoop},
    window::{Window, WindowId},
};

/// Window manager for the point cloud viewer
pub struct WindowManager {
    /// Window configuration
    config: ViewerConfig,

    /// The actual window (created during event loop)
    window: Option<Arc<Window>>,

    /// Event handlers
    event_handlers: Vec<Box<dyn EventHandler>>,
}

/// Trait for handling window events
pub trait EventHandler: Send + Sync {
    /// Handle window events
    fn handle_event(&mut self, event: &WindowEvent, window: &Window) -> bool;

    /// Handle redraw requests
    fn handle_redraw(&mut self, window: &Window) -> Result<()>;

    /// Handle window resize
    fn handle_resize(&mut self, new_size: winit::dpi::PhysicalSize<u32>) -> Result<()>;

    /// Check if the handler is initialized
    fn is_initialized(&self) -> bool;

    /// Initialize the handler (called on first event)
    fn try_initialize(&mut self, window: Arc<Window>) -> Result<()>;
}

impl WindowManager {
    /// Create a new window manager with the given configuration
    pub fn new(config: ViewerConfig) -> Self {
        Self {
            config,
            window: None,
            event_handlers: Vec::new(),
        }
    }

    /// Add an event handler
    pub fn add_event_handler(&mut self, handler: Box<dyn EventHandler>) {
        self.event_handlers.push(handler);
    }

    /// Run the window event loop
    pub async fn run(self) -> Result<()> {
        let event_loop = EventLoop::new().map_err(|e| {
            CloudError::Visualization(format!("Failed to create event loop: {}", e))
        })?;

        event_loop.set_control_flow(ControlFlow::Poll);

        let mut app = WindowApp {
            manager: self,
            should_close: false,
        };

        event_loop
            .run_app(&mut app)
            .map_err(|e| CloudError::Visualization(format!("Event loop error: {}", e)))?;

        Ok(())
    }

    /// Get the window reference
    pub fn window(&self) -> Option<&Arc<Window>> {
        self.window.as_ref()
    }

    /// Create the window
    fn create_window(&mut self, event_loop: &ActiveEventLoop) -> Result<()> {
        let window_attributes = Window::default_attributes()
            .with_title(&self.config.title)
            .with_inner_size(winit::dpi::LogicalSize::new(
                self.config.width,
                self.config.height,
            ))
            .with_resizable(self.config.resizable);

        let window = event_loop
            .create_window(window_attributes)
            .map_err(|e| CloudError::Visualization(format!("Failed to create window: {}", e)))?;

        self.window = Some(Arc::new(window));
        Ok(())
    }
}

/// Application handler for the window event loop
struct WindowApp {
    manager: WindowManager,
    should_close: bool,
}

impl ApplicationHandler for WindowApp {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.manager.window.is_none() {
            if let Err(e) = self.manager.create_window(event_loop) {
                eprintln!("Failed to create window: {}", e);
                self.should_close = true;
                return;
            }
        }
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        _window_id: WindowId,
        event: WindowEvent,
    ) {
        if let Some(window) = &self.manager.window {
            match &event {
                WindowEvent::CloseRequested => {
                    self.should_close = true;
                }
                WindowEvent::Resized(new_size) => {
                    for handler in &mut self.manager.event_handlers {
                        if let Err(e) = handler.handle_resize(*new_size) {
                            eprintln!("Error handling resize: {}", e);
                        }
                    }
                }
                WindowEvent::RedrawRequested => {
                    for handler in &mut self.manager.event_handlers {
                        if let Err(e) = handler.handle_redraw(window) {
                            eprintln!("Error during redraw: {}", e);
                        }
                    }
                }
                _ => {
                    // Initialize handlers if needed and pass event to all handlers
                    for handler in &mut self.manager.event_handlers {
                        if !handler.is_initialized() {
                            if let Err(e) = handler.try_initialize(window.clone()) {
                                eprintln!("Failed to initialize handler: {}", e);
                                continue;
                            }
                        }
                        handler.handle_event(&event, window);
                    }
                }
            }
        }

        if self.should_close {
            event_loop.exit();
        }
    }

    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        // Request redraw for continuous rendering
        if let Some(window) = &self.manager.window {
            window.request_redraw();
        }
    }
}

/// Builder for creating window managers with custom configurations
pub struct WindowManagerBuilder {
    config: ViewerConfig,
}

impl WindowManagerBuilder {
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

    /// Build the window manager
    pub fn build(self) -> WindowManager {
        WindowManager::new(self.config)
    }
}

impl Default for WindowManagerBuilder {
    fn default() -> Self {
        Self::new()
    }
}
