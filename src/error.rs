//! Error types and result handling for FerrumCloud
//!
//! This module provides comprehensive error handling using `thiserror` for
//! structured error types and `anyhow` for error context.

use thiserror::Error;

/// Result type alias for FerrumCloud operations
pub type Result<T> = std::result::Result<T, CloudError>;

/// Main error type for FerrumCloud operations
#[derive(Error, Debug)]
pub enum CloudError {
    /// I/O related errors
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    /// Invalid point cloud format
    #[error("Invalid point cloud format: {0}")]
    FormatError(String),

    /// Algorithm execution failed
    #[error("Algorithm failed: {0}")]
    AlgorithmError(String),

    /// Invalid parameters provided
    #[error("Invalid parameter: {0}")]
    InvalidParameter(String),

    /// Memory allocation error
    #[error("Memory allocation failed: {0}")]
    MemoryError(String),

    /// Search operation failed
    #[error("Search operation failed: {0}")]
    SearchError(String),

    /// Serialization/deserialization error
    #[error("Serialization error: {0}")]
    SerializationError(#[from] serde_json::Error),

    /// Visualization related errors
    #[cfg(feature = "visualization")]
    #[error("Visualization error: {0}")]
    Visualization(String),

    /// Generic error with custom message
    #[error("{0}")]
    Custom(String),
}

impl CloudError {
    /// Create a new format error
    pub fn format_error<S: Into<String>>(msg: S) -> Self {
        CloudError::FormatError(msg.into())
    }

    /// Create a new algorithm error
    pub fn algorithm_error<S: Into<String>>(msg: S) -> Self {
        CloudError::AlgorithmError(msg.into())
    }

    /// Create a new invalid parameter error
    pub fn invalid_parameter<S: Into<String>>(msg: S) -> Self {
        CloudError::InvalidParameter(msg.into())
    }

    /// Create a new memory error
    pub fn memory_error<S: Into<String>>(msg: S) -> Self {
        CloudError::MemoryError(msg.into())
    }

    /// Create a new search error
    pub fn search_error<S: Into<String>>(msg: S) -> Self {
        CloudError::SearchError(msg.into())
    }

    /// Create a visualization error
    #[cfg(feature = "visualization")]
    pub fn visualization_error<S: Into<String>>(msg: S) -> Self {
        CloudError::Visualization(msg.into())
    }

    /// Create a custom error
    pub fn custom<S: Into<String>>(msg: S) -> Self {
        CloudError::Custom(msg.into())
    }
}
