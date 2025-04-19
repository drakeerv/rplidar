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
