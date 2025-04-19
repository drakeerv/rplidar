/// Calculates the 8-bit XOR checksum used in some RPLIDAR protocol commands.
pub struct Checksum {
    current: u8,
}

impl Checksum {
    /// Creates a new `Checksum` instance, initialized to 0.
    #[inline]
    pub fn new() -> Checksum {
        Checksum { current: 0 }
    }

    /// Includes a slice of bytes in the checksum calculation.
    ///
    /// # Arguments
    ///
    /// * `data` - The byte slice to XOR into the current checksum.
    #[inline]
    pub fn push_slice(&mut self, data: &[u8]) {
        for d in data {
            self.current ^= d;
        }
    }

    /// Returns the calculated checksum value.
    #[inline]
    pub fn checksum(&self) -> u8 {
        self.current
    }
}
