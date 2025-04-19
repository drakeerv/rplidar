use crate::base::error::{Error, Result}; // Updated
use crate::base::message::Message; // Updated
use crate::base::ring_byte_buffer::RingByteBuffer;
use crate::base::traits::{ProtocolDecoder, ProtocolEncoder}; // Updated
use log::{error, trace, warn};
use std::io;
use std::time::{Duration, Instant};

const DEFAULT_CHANNEL_READ_BUFFER_SIZE: usize = 1024;

/// Channel encode and decode message with protocol, and send and receive bytes via stream
///
/// # Examples
/// ```ignore
/// let mut channel = Channel::new(
///     RplidarProtocol::new(),
///     serial_port
/// );
///
/// channel.write(&Message::new(1)).unwrap();
/// ```
#[derive(Debug)]
pub struct Channel<P, T: ?Sized> {
    protocol: P,
    stream: Box<T>,
    read_buffer: RingByteBuffer,
}

impl<P, T: ?Sized> Channel<P, T>
where
    P: ProtocolDecoder + ProtocolEncoder,
    T: io::Read + io::Write,
{
    /// Create a new `Channel` to read and write messages
    ///
    /// # Example
    /// ```ignore
    /// let channel = Channel::new(
    ///     RplidarProtocol::new(),
    ///     serial_port
    /// );
    /// ```
    pub fn new(protocol: P, stream: Box<T>) -> Channel<P, T> {
        trace!(
            "Creating new Channel with default buffer size {}",
            DEFAULT_CHANNEL_READ_BUFFER_SIZE
        );
        Channel::with_read_buffer_size(protocol, stream, DEFAULT_CHANNEL_READ_BUFFER_SIZE)
    }

    /// Create a new `Channel` with non-default ring buffer capacity
    ///
    /// # Example
    /// ```ignore
    /// let channel = Channel::with_read_buffer_size(
    ///     RplidarProtocol::new(),
    ///     serial_port,
    ///     100000 as usize
    /// );
    /// ```
    pub fn with_read_buffer_size(
        protocol: P,
        stream: Box<T>,
        read_buffer_size: usize,
    ) -> Channel<P, T> {
        trace!("Creating new Channel with buffer size {}", read_buffer_size);
        let mut chn = Channel {
            protocol,
            stream,
            read_buffer: RingByteBuffer::with_capacity(read_buffer_size),
        };

        chn.reset(); // Reset calls trace internally
        chn
    }

    /// Reset the channel status
    /// This function is usually used to reset protocol encoder and decoder when meet communication error
    ///
    /// # Example
    /// ```ignore
    /// match channel.invoke(&Message::new(1), Duration::from_secs(1)) {
    ///     Ok(_) => {}.
    ///     Err(_) => { channel.reset(); }
    /// }
    /// ```
    pub fn reset(&mut self) {
        trace!("Resetting Channel protocol encoder and decoder");
        self.protocol.reset_encoder();
        self.protocol.reset_decoder();
        // Consider if buffer should be cleared too? Depends on desired behavior after reset.
        // self.read_buffer.clear(); // Optional: uncomment if buffer should be cleared on reset
    }

    /// Read message from channel
    ///
    /// # Example
    /// ```ignore
    /// if let Some(msg) = channel.read().unwrap() {
    ///     println!("{:?}", msg);
    /// }
    /// ```
    pub fn read(&mut self) -> Result<Option<Message>> {
        trace!("Channel read called");
        loop {
            trace!(
                "Attempting to read from stream into buffer (current buffer len: {})",
                self.read_buffer.len()
            );
            match self.read_buffer.read_from(&mut self.stream) {
                Ok(bytes_read) => trace!("Read {} bytes from stream", bytes_read),
                Err(e) if e.kind() == io::ErrorKind::TimedOut => {
                    trace!("Stream read timed out (non-blocking likely)");
                    // This is expected if stream has timeout and no data is ready
                }
                Err(e) => {
                    error!("IO error reading from stream: {}", e);
                    return Err(e.into());
                }
            }

            let buffer_slice = self.read_buffer.current_read_slice();
            trace!(
                "Attempting to decode buffer ({} bytes available)",
                buffer_slice.len()
            );
            if buffer_slice.is_empty() {
                trace!("Buffer is empty, returning None");
                return Ok(None); // No data to decode
            }

            match self.protocol.decode(buffer_slice) {
                Ok((decoded_bytes, msg_option)) => {
                    trace!(
                        "Protocol decode consumed {} bytes, message found: {}",
                        decoded_bytes,
                        msg_option.is_some()
                    );
                    if decoded_bytes > 0 {
                        self.read_buffer.skip_bytes(decoded_bytes);
                        trace!(
                            "Skipped {} bytes from buffer (new len: {})",
                            decoded_bytes,
                            self.read_buffer.len()
                        );
                    }

                    if let Some(msg) = msg_option {
                        trace!(
                            "Decoded message: cmd={:02X}, data_len={}",
                            msg.cmd,
                            msg.data.len()
                        );
                        return Ok(Some(msg));
                    }

                    // If decode consumed 0 bytes, it means we need more data or there's a persistent protocol error
                    // Use self.read_buffer.len() instead of !buffer_slice.is_empty() to avoid holding the borrow
                    if decoded_bytes == 0 && !self.read_buffer.is_empty() {
                        warn!("Protocol decode consumed 0 bytes with non-empty buffer ({} bytes). Possible protocol error or need more data.", self.read_buffer.len());
                        // If the buffer is full and decode still consumes 0, it's likely an error state.
                        if self.read_buffer.free_space() == 0 {
                            error!("Buffer full and decode consumed 0 bytes. Resetting decoder.");
                            self.protocol.reset_decoder();
                            // Optionally clear buffer here too if needed
                            // self.read_buffer.clear();
                            return Err(Error::ProtocolError {
                                description: "Decoder stalled on full buffer".to_owned(),
                            });
                        }
                        // Otherwise, assume more data is needed and loop will try reading again.
                        // If stream read keeps returning 0, we might loop indefinitely without timeout.
                        // The read_until logic handles overall timeout.
                        // For read(), we might need a mechanism to prevent infinite loops if stream never provides more data.
                        // However, typical use is read_until or within a select/poll loop.
                        // If we simply return None here, the caller might loop indefinitely too.
                        // Let's return Ok(None) for now, assuming caller handles potential loops or uses read_until.
                        return Ok(None);
                    }

                    // If decode consumed bytes but no message, loop continues to read more data
                }
                Err(e) => {
                    error!("Protocol decode error: {:?}", e);
                    self.protocol.reset_decoder(); // Reset decoder on error
                    return Err(e);
                }
            }
        }
    }

    /// Read message until timeout
    ///
    /// # Example
    /// ```ignore
    /// channel.read_until(Duration::from_secs(1));
    /// ```
    pub fn read_until(&mut self, timeout: Duration) -> Result<Option<Message>> {
        trace!("Channel read_until called with timeout {:?}", timeout);
        let start = Instant::now();

        loop {
            // 1. Try decoding existing data first
            let buffer_slice = self.read_buffer.current_read_slice();
            if !buffer_slice.is_empty() {
                // Removed parentheses
                trace!(
                    "Attempting to decode existing buffer data ({} bytes)",
                    buffer_slice.len()
                );
                match self.protocol.decode(buffer_slice) {
                    Ok((decoded_bytes, msg_option)) => {
                        trace!(
                            "Protocol decode consumed {} bytes, message found: {}",
                            decoded_bytes,
                            msg_option.is_some()
                        );
                        if decoded_bytes > 0 {
                            self.read_buffer.skip_bytes(decoded_bytes);
                            trace!(
                                "Skipped {} bytes from buffer (new len: {})",
                                decoded_bytes,
                                self.read_buffer.len()
                            );
                        }
                        if let Some(msg) = msg_option {
                            trace!(
                                "Decoded message: cmd={:02X}, data_len={}",
                                msg.cmd,
                                msg.data.len()
                            );
                            return Ok(Some(msg));
                        }
                        // If decode consumed 0 bytes with data present, log warning but continue to read more
                        // Use self.read_buffer.len() instead of !buffer_slice.is_empty() to avoid holding the borrow
                        if decoded_bytes == 0 && !self.read_buffer.is_empty() {
                            warn!("Protocol decode consumed 0 bytes with non-empty buffer ({} bytes) in read_until. Possible protocol error or need more data.", self.read_buffer.len());
                            // Check if buffer is full to detect potential stall
                            if self.read_buffer.free_space() == 0 {
                                error!("Buffer full and decode consumed 0 bytes in read_until. Resetting decoder.");
                                self.protocol.reset_decoder();
                                return Err(Error::ProtocolError {
                                    description: "Decoder stalled on full buffer in read_until"
                                        .to_owned(),
                                });
                            }
                        }
                    }
                    Err(e) => {
                        error!("Protocol decode error: {:?}", e);
                        self.protocol.reset_decoder(); // Reset decoder on error
                        return Err(e);
                    }
                }
            } else {
                trace!("Buffer is empty, proceeding to read from stream");
            }

            // 2. Check timeout *before* potentially blocking read
            let elapsed = start.elapsed();
            if elapsed >= timeout {
                // Removed parentheses
                trace!("Overall timeout reached ({:?})", elapsed);
                return Err(Error::OperationTimeout);
            }

            // 3. Try reading more data from the stream
            // The stream should have a short timeout configured (e.g., 1ms)
            // so this doesn't block for the entire remaining duration.
            trace!(
                "Attempting to read from stream (remaining time: {:?})",
                timeout.saturating_sub(elapsed)
            );
            match self.read_buffer.read_from(&mut self.stream) {
                Ok(0) => {
                    trace!("Stream read returned 0 bytes (likely timeout or no data)");
                    // No new data, loop will check overall timeout and try decode again if buffer wasn't empty
                }
                Ok(bytes_read) => {
                    trace!(
                        "Read {} bytes from stream (new buffer len: {})",
                        bytes_read,
                        self.read_buffer.len()
                    );
                    // Data read, loop will attempt decode again
                }
                Err(e) if e.kind() == io::ErrorKind::TimedOut => {
                    trace!("Stream read timed out (explicitly)");
                    // Stream timeout is expected, continue checking overall timeout
                }
                Err(e) => {
                    error!("IO error reading from stream: {}", e);
                    return Err(e.into());
                }
            }

            // Loop continues: checks decode again, then timeout, then reads again.
        }
    }

    /// Write message to channel
    ///
    /// # Example
    /// ```ignore
    /// channel.write(&Message::new(1)).unwrap();
    /// ```
    pub fn write(&mut self, msg: &Message) -> Result<usize> {
        trace!(
            "Channel write called: cmd={:02X}, data_len={}",
            msg.cmd,
            msg.data.len()
        );
        let written = self.protocol.write_to(msg, &mut self.stream)?; // write_to logs internally
        trace!("Flushing stream...");
        self.stream.flush()?;
        trace!("Stream flushed");
        Ok(written)
    }

    /// Send a request to channel and wait for response
    ///
    /// # Example
    /// ```ignore
    /// let resp = channel.invoke(&Message::new(1), Duration::from_secs(1));
    /// ```
    pub fn invoke(&mut self, request: &Message, timeout: Duration) -> Result<Option<Message>> {
        trace!(
            "Channel invoke called: cmd={:02X}, data_len={}, timeout={:?}",
            request.cmd,
            request.data.len(),
            timeout
        );
        match self.write(request) {
            Ok(written) => trace!("Invoke: wrote {} bytes for request", written),
            Err(e) => {
                error!("Invoke: failed to write request: {:?}", e);
                return Err(e);
            }
        }
        trace!("Invoke: waiting for response...");
        let result = self.read_until(timeout); // read_until logs internally
        match &result {
            Ok(Some(resp)) => trace!(
                "Invoke: received response: cmd={:02X}, data_len={}",
                resp.cmd,
                resp.data.len()
            ),
            Ok(None) => warn!("Invoke: timed out waiting for response"),
            Err(e) => error!("Invoke: error waiting for response: {:?}", e),
        }
        result
    }
}
