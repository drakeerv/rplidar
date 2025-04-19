use std::io;

// Re-export error types for convenience.
pub use crate::base::error::*;

/// Represents a command or response message exchanged with the RPLIDAR device.
#[derive(Debug, Clone, PartialEq)]
pub struct Message {
    /// The command code or response type identifier.
    pub cmd: u8,

    /// Optional payload data associated with the message.
    pub data: Vec<u8>,
}

impl Message {
    /// Creates a new message with a command code and no payload.
    ///
    /// # Arguments
    ///
    /// * `cmd` - The command code for the message.
    pub fn new(cmd: u8) -> Message {
        Message::with_data(cmd, &[])
    }

    /// Creates a new message with a command code and payload data.
    ///
    /// # Arguments
    ///
    /// * `cmd` - The command code for the message.
    /// * `data` - A slice containing the payload data.
    #[inline]
    pub fn with_data(cmd: u8, data: &[u8]) -> Message {
        Message {
            cmd,
            data: data.to_vec(),
        }
    }
}

/// Defines the behavior for decoding byte streams into `Message` objects.
pub trait ProtocolDecoder {
    /// Attempts to decode a `Message` from the provided buffer.
    ///
    /// Returns a `Result` containing a tuple:
    /// * The number of bytes consumed from the buffer.
    /// * An `Option<Message>` which is `Some` if a complete message was decoded, or `None` otherwise.
    ///
    /// # Arguments
    ///
    /// * `buf` - The byte slice containing the data to decode.
    fn decode(&mut self, buf: &[u8]) -> Result<(usize, Option<Message>)>;

    /// Resets the internal state of the decoder.
    /// This is typically called after a communication error or when starting a new session.
    fn reset_decoder(&mut self);
}

/// Defines the behavior for encoding `Message` objects into byte streams.
pub trait ProtocolEncoder {
    /// Encodes a `Message` into the provided byte buffer.
    ///
    /// Returns the number of bytes written to the buffer upon successful encoding.
    ///
    /// # Arguments
    ///
    /// * `msg` - The `Message` to encode.
    /// * `bytes` - The mutable byte slice to write the encoded message into.
    fn encode(&mut self, msg: &Message, bytes: &mut [u8]) -> Result<usize>;

    /// Estimates the maximum size in bytes required to encode the given `Message`.
    /// The actual encoded size must be less than or equal to this estimate.
    ///
    /// # Arguments
    ///
    /// * `msg` - The `Message` for which to estimate the encoded size.
    fn estimate_encoded_size(&mut self, msg: &Message) -> Result<usize>;

    /// Encodes a `Message` and writes it directly to a `Write` target (e.g., a serial port).
    ///
    /// Returns the number of bytes successfully written to the destination.
    ///
    /// # Arguments
    ///
    /// * `msg` - The `Message` to encode and write.
    /// * `dest` - The `Write` target to write the encoded bytes to.
    fn write_to(&mut self, msg: &Message, dest: &mut impl io::Write) -> Result<usize>;

    /// Resets the internal state of the encoder.
    fn reset_encoder(&mut self);
}
