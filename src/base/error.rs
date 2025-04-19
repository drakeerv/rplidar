use std::error;
use std::fmt;
use std::io;

/// Represents errors that can occur during RPLIDAR operations.
#[derive(Debug)]
pub enum Error {
    /// The execution of operation failed. Contains a description of the failure.
    OperationFail { description: String },

    /// The execution of operation is timed out.
    OperationTimeout,

    /// The device doesn't support this operation.
    OperationNotSupport,

    /// The decoding data is invalid according to current protocol. Contains a description of the protocol error.
    ProtocolError { description: String },

    /// The buffer provided is too small for message encoding.
    BufferTooSmall,

    /// An I/O error occurred while communicating with the underlying stream (e.g., serial port).
    IoError(io::Error),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::OperationFail { description } => write!(f, "operation failed: {}", description),
            Error::OperationTimeout => write!(f, "operation timeout"),
            Error::OperationNotSupport => write!(f, "operation not support"),
            Error::ProtocolError { description } => write!(f, "protocol error: {}", description),
            Error::BufferTooSmall => write!(f, "buffer is too small for message encoding"),
            Error::IoError(err) => write!(f, "io error: {}", err),
        }
    }
}

impl error::Error for Error {}

impl From<io::Error> for Error {
    fn from(err: io::Error) -> Self {
        Error::IoError(err)
    }
}

/// A specialized `Result` type for RPLIDAR operations.
pub type Result<T> = std::result::Result<T, Error>;
