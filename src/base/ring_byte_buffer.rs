use std::cmp::min;
use std::io::{Read, Write};

/// A ring byte buffer used for efficient reading and writing of byte streams.
///
/// This buffer allows writing data into it and reading data out of it in a circular manner.
/// It's useful for managing data flow between producers and consumers, especially with I/O operations.
///
/// # Example
///
/// ```rust
/// # use std::io::Write;
/// // Use the correct path based on your crate structure
/// # use rplidar::driver::RingByteBuffer;
/// let mut buffer = RingByteBuffer::with_capacity(100);
/// buffer.write(&[0, 1, 2, 3]).unwrap();
/// assert_eq!(buffer.len(), 4 as usize);
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct RingByteBuffer {
    buf: Vec<u8>,
    head: usize,
    size: usize,
}

impl RingByteBuffer {
    /// Creates a new `RingByteBuffer` with the specified capacity.
    ///
    /// # Arguments
    ///
    /// * `capacity` - The maximum number of bytes the buffer can hold.
    pub fn with_capacity(capacity: usize) -> RingByteBuffer {
        RingByteBuffer {
            buf: vec![0; capacity],
            head: 0,
            size: 0,
        }
    }

    /// Returns the number of bytes currently stored in the buffer.
    pub fn len(&self) -> usize {
        self.size
    }

    /// Returns `true` if the buffer contains no bytes.
    pub fn is_empty(&self) -> bool {
        self.size == 0
    }

    /// Returns the total capacity of the buffer in bytes.
    pub fn capacity(&self) -> usize {
        self.buf.len()
    }

    /// Returns the amount of free space available in the buffer in bytes.
    pub fn free_space(&self) -> usize {
        self.buf.len() - self.size
    }

    /// current tail index of the ring buffer
    fn tail(&self) -> usize {
        (self.head + self.size) % self.buf.len()
    }

    /// Returns a slice representing the contiguous readable portion of the buffer.
    ///
    /// This slice contains the oldest data written to the buffer that hasn't been read yet.
    /// The slice might not contain all readable data if the data wraps around the end of the internal buffer.
    ///
    /// # Example
    /// ```rust
    /// # use std::io::{ stdout, Write };
    /// // Use the correct path based on your crate structure
    /// # use rplidar::driver::RingByteBuffer;
    /// # let mut buffer = RingByteBuffer::with_capacity(100);
    /// # let mut some_stream = stdout();
    /// let read_slice = buffer.current_read_slice();
    /// let read = some_stream.write(read_slice).unwrap();
    /// buffer.skip_bytes(read);
    /// ```
    pub fn current_read_slice(&self) -> &[u8] {
        let end = min(self.head + self.size, self.buf.len());
        &self.buf[self.head..end]
    }

    /// Removes the specified number of bytes from the beginning of the readable data.
    ///
    /// Returns the actual number of bytes skipped, which may be less than `bytes` if the buffer contains fewer bytes.
    ///
    /// # Arguments
    ///
    /// * `bytes` - The maximum number of bytes to skip.
    pub fn skip_bytes(&mut self, bytes: usize) -> usize {
        let skipped = min(self.size, bytes);
        self.head = (self.head + skipped) % self.buf.len();
        self.size -= skipped;
        skipped
    }

    /// current write slice
    fn current_write_slice(&mut self) -> &mut [u8] {
        let current_end = self.tail();
        let write_buf_end = min(self.buf.len(), current_end + self.free_space());
        &mut self.buf[current_end..write_buf_end]
    }

    /// Updates the internal state to reflect that `bytes` have been written into the buffer.
    /// This should be called after writing data into the slice returned by `current_write_slice`.
    ///
    /// # Arguments
    ///
    /// * `bytes` - The number of bytes written.
    pub fn mark_bytes_as_written(&mut self, bytes: usize) {
        let written = min(self.free_space(), bytes);
        self.size += written;
    }

    fn partial_read_from(&mut self, upstream: &mut impl Read) -> std::io::Result<usize> {
        if self.current_write_slice().is_empty() {
            return Ok(0);
        }

        match upstream.read(self.current_write_slice()) {
            Ok(read) => {
                self.mark_bytes_as_written(read);
                Ok(read)
            }
            Err(err) => {
                if err.kind() == std::io::ErrorKind::TimedOut {
                    Ok(0)
                } else {
                    Err(err)
                }
            }
        }
    }

    /// Reads data from an upstream source (`Read` trait) and fills the buffer.
    ///
    /// This method attempts to fill the buffer as much as possible from the upstream source.
    /// It handles potential wrap-around scenarios within the ring buffer.
    /// Returns the total number of bytes read from the upstream source.
    ///
    /// # Example
    /// ```rust
    /// # use std::io::{ stdin, Read };
    /// // Use the correct path based on your crate structure
    /// # use rplidar::driver::RingByteBuffer;
    /// # let mut buffer = RingByteBuffer::with_capacity(100);
    /// # let mut some_stream = stdin();
    /// let read = buffer.read_from(&mut some_stream).unwrap();
    /// println!("{} bytes read from stream to fill buffer, current length of buffer is: {}", read, buffer.len());
    /// ```
    pub fn read_from(&mut self, upstream: &mut impl Read) -> std::io::Result<usize> {
        let read = self.partial_read_from(upstream)?;

        match self.partial_read_from(upstream) {
            Ok(latter_read) => Ok(read + latter_read),
            Err(err) => Err(err),
        }
    }
}

impl Read for RingByteBuffer {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        let read = {
            let current_read_slice = self.current_read_slice();
            let read = min(current_read_slice.len(), buf.len());
            buf[0..read].clone_from_slice(&current_read_slice[0..read]);

            read
        };
        self.skip_bytes(read);

        let latter_read = {
            let current_read_slice = self.current_read_slice();
            let latter_read = min(current_read_slice.len(), buf.len() - read);
            buf[read..read + latter_read].clone_from_slice(&current_read_slice[0..latter_read]);

            latter_read
        };
        self.skip_bytes(latter_read);

        Ok(read + latter_read)
    }
}

impl Write for RingByteBuffer {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        let written = {
            let current_write_slice = self.current_write_slice();
            let written = min(current_write_slice.len(), buf.len());
            current_write_slice[0..written].clone_from_slice(&buf[0..written]);
            written
        };
        self.mark_bytes_as_written(written);

        let latter_written = {
            let current_write_slice = self.current_write_slice();
            let latter_written = min(current_write_slice.len(), buf.len() - written);
            current_write_slice[0..latter_written]
                .clone_from_slice(&buf[written..written + latter_written]);
            latter_written
        };
        self.mark_bytes_as_written(latter_written);

        Ok(written + latter_written)
    }

    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}
