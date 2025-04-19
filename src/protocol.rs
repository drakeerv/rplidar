use crate::base::{Error, Message, ProtocolDecoder, ProtocolEncoder, Result}; // Use re-exports from base
use crate::checksum::Checksum;
use byteorder::{ByteOrder, LittleEndian};
use log::error;
use log::trace;
use std::cmp::min;
use std::io::Write;

const RPLIDAR_CMD_SYNC_BYTE: u8 = 0xA5;
const RPLIDAR_CMDFLAG_HAS_PAYLOAD: u8 = 0x80;

const RPLIDAR_ANS_SYNC_BYTES: [u8; 2] = [0xA5, 0x5A];

const RPLIDAR_ANS_PKTFLAG_LOOP: u8 = 0x1;

const RPLIDAR_ANS_HEADER_SIZE_MASK: u32 = 0x3FFFFFFF;
const RPLIDAR_ANS_HEADER_SUBTYPE_SHIFT: usize = 30;

/// The size of RPLIDAR protocol answer header (not including the two sync bytes)
const RPLIDAR_ANS_HEADER_SIZE: usize = 5;

#[derive(Debug, Clone, PartialEq)]
enum DecodeStatus {
    WaitSyncByte(usize),
    WaitAnsHeader,
    ReceiveResponse,
}

/// The implementation of the RPLIDAR host communication protocol.
///
/// This struct handles encoding commands (`Message` -> bytes) and decoding responses (bytes -> `Message`)
/// according to the RPLIDAR serial communication protocol specification.
#[derive(Debug, Clone, PartialEq)]
pub struct RplidarHostProtocol {
    status: DecodeStatus,
    ans_header: Vec<u8>,
    ans_flag: u8,
    response_size: usize,
    decoding_msg: Message,
}

impl RplidarHostProtocol {
    /// Creates a new `RplidarHostProtocol` instance in its initial state.
    pub fn new() -> RplidarHostProtocol {
        trace!("Creating new RplidarHostProtocol");
        RplidarHostProtocol {
            status: DecodeStatus::WaitSyncByte(0),
            ans_header: Vec::new(),
            ans_flag: 0,
            response_size: 0,
            decoding_msg: Message::new(0),
        }
    }

    fn start_wait_sync_bytes(&mut self, sync_byte_index: usize) {
        trace!("Decoder state -> WaitSyncByte({})", sync_byte_index);
        self.status = DecodeStatus::WaitSyncByte(sync_byte_index);
        self.ans_header.clear();
        self.ans_flag = 0;
        self.response_size = 0;
    }

    fn start_wait_ans_header(&mut self) {
        trace!("Decoder state -> WaitAnsHeader");
        self.status = DecodeStatus::WaitAnsHeader;
        self.ans_header.clear();
        self.ans_flag = 0;
        self.response_size = 0;
    }

    fn start_receive_response(&mut self) {
        trace!(
            "Decoder state -> ReceiveResponse (size: {})",
            self.response_size
        );
        self.status = DecodeStatus::ReceiveResponse;
        self.decoding_msg.data.clear();
        self.decoding_msg.data.reserve(self.response_size); // Pre-allocate buffer
    }

    fn decode_sync_bytes(&mut self, buf: &[u8]) -> Result<usize> {
        trace!("Decoding sync byte, current state: {:?}", self.status);
        if let DecodeStatus::WaitSyncByte(i) = self.status {
            let current_byte = buf[0];
            trace!(
                "Received byte {:02X}, expecting sync byte {} ({:02X})",
                current_byte,
                i,
                RPLIDAR_ANS_SYNC_BYTES[i]
            );
            if current_byte != RPLIDAR_ANS_SYNC_BYTES[i] {
                trace!("Sync byte mismatch, resetting to WaitSyncByte(0)");
                self.start_wait_sync_bytes(0);
                Ok(1)
            } else if i == RPLIDAR_ANS_SYNC_BYTES.len() - 1 {
                trace!("All sync bytes received, moving to WaitAnsHeader");
                self.start_wait_ans_header();
                Ok(1)
            } else {
                trace!("Sync byte {} OK, moving to WaitSyncByte({})", i, i + 1);
                self.start_wait_sync_bytes(i + 1);
                Ok(1)
            }
        } else {
            // This should not happen if decode() logic is correct
            error!(
                "decode_sync_bytes called in incorrect state: {:?}",
                self.status
            );
            Err(Error::ProtocolError {
                description: "sync byte status error".to_owned(),
            })
        }
    }

    fn decode_ans_header(&mut self, buf: &[u8]) -> Result<(usize, Option<Message>)> {
        trace!("Decoding answer header, current state: {:?}", self.status);
        let bytes_to_read = RPLIDAR_ANS_HEADER_SIZE - self.ans_header.len();
        let bytes_actual_read = min(bytes_to_read, buf.len());
        trace!(
            "Need {} header bytes, received {}, reading {}",
            bytes_to_read,
            buf.len(),
            bytes_actual_read
        );
        self.ans_header
            .extend_from_slice(&buf[0..bytes_actual_read]);
        trace!("Header buffer size: {}", self.ans_header.len());

        if self.ans_header.len() == RPLIDAR_ANS_HEADER_SIZE {
            trace!("Full header received: {:?}", self.ans_header);
            self.decode_ans_header_metadata();
            trace!(
                "Decoded header metadata: cmd={:02X}, flag={:02X}, size={}",
                self.decoding_msg.cmd,
                self.ans_flag,
                self.response_size
            );
            if self.response_size == 0 {
                if (self.ans_flag & RPLIDAR_ANS_PKTFLAG_LOOP) == RPLIDAR_ANS_PKTFLAG_LOOP {
                    error!(
                        "Received loop answer with zero response size, flag={:02X}",
                        self.ans_flag
                    );
                    Err(Error::ProtocolError {
                        description: "received loop answer with no response size".to_owned(),
                    })
                } else {
                    trace!("Response size is 0, returning message immediately");
                    let answer = Ok((bytes_actual_read, Some(self.decoding_msg.clone())));
                    self.reset_decoder(); // Reset state after returning message
                    answer
                }
            } else {
                trace!(
                    "Response size is {}, moving to ReceiveResponse state",
                    self.response_size
                );
                self.start_receive_response();
                Ok((bytes_actual_read, None))
            }
        } else {
            trace!("Header incomplete, need more data");
            Ok((bytes_actual_read, None))
        }
    }

    fn decode_response(&mut self, buf: &[u8]) -> Result<(usize, Option<Message>)> {
        trace!("Decoding response data, current state: {:?}", self.status);
        let bytes_to_read = self.response_size - self.decoding_msg.data.len();
        let bytes_actual_read = min(bytes_to_read, buf.len());
        trace!(
            "Need {} response bytes, received {}, reading {}",
            bytes_to_read,
            buf.len(),
            bytes_actual_read
        );
        self.decoding_msg
            .data
            .extend_from_slice(&buf[0..bytes_actual_read]);
        trace!(
            "Response data buffer size: {}",
            self.decoding_msg.data.len()
        );

        if self.decoding_msg.data.len() == self.response_size {
            trace!("Full response data received");
            let answer = Ok((bytes_actual_read, Some(self.decoding_msg.clone())));
            if (self.ans_flag & RPLIDAR_ANS_PKTFLAG_LOOP) == RPLIDAR_ANS_PKTFLAG_LOOP {
                trace!("Loop flag set, staying in ReceiveResponse state for next packet");
                self.start_receive_response(); // Prepare for next loop packet
            } else {
                trace!("Loop flag not set, resetting decoder state");
                self.reset_decoder(); // Reset state after returning message
            }
            answer
        } else {
            trace!("Response data incomplete, need more data");
            Ok((bytes_actual_read, None))
        }
    }

    /// when we finished reading the first five bytes of answer header, we will know the message type and payload size of answer header
    fn decode_ans_header_metadata(&mut self) {
        // This function is called internally by decode_ans_header, logging is done there
        self.decoding_msg = Message::new(self.ans_header[4]);
        let size_q30_subtype = LittleEndian::read_u32(&self.ans_header[0..4]);
        self.ans_flag = (size_q30_subtype >> RPLIDAR_ANS_HEADER_SUBTYPE_SHIFT as u32) as u8;
        self.response_size = (size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK) as usize;
    }
}

impl Default for RplidarHostProtocol {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for RplidarHostProtocol {
    /// Decodes bytes from the input buffer according to the RPLIDAR protocol.
    ///
    /// Maintains internal state to handle multi-byte messages and synchronization.
    /// Returns the number of bytes consumed and an optional decoded `Message`.
    fn decode(&mut self, buf: &[u8]) -> Result<(usize, Option<Message>)> {
        trace!(
            "decode called with {} bytes, current state: {:?}",
            buf.len(),
            self.status
        );
        if buf.is_empty() {
            trace!("decode buffer is empty, returning (0, None)");
            return Ok((0, None));
        }

        let mut i = 0;
        while i < buf.len() {
            let current_byte_for_log = buf[i]; // For logging
            let initial_state_for_log = self.status.clone(); // For logging
            trace!(
                "Processing byte {} ({:02X}) at index {}, state: {:?}",
                i,
                current_byte_for_log,
                i,
                initial_state_for_log
            );

            match self.status {
                DecodeStatus::WaitSyncByte(_) => {
                    match self.decode_sync_bytes(&buf[i..]) {
                        Ok(consumed) => {
                            trace!("decode_sync_bytes consumed {}", consumed);
                            i += consumed;
                        }
                        Err(e) => {
                            error!("Error in decode_sync_bytes: {:?}", e);
                            self.reset_decoder(); // Reset on error
                            return Err(e);
                        }
                    }
                }
                DecodeStatus::WaitAnsHeader => {
                    match self.decode_ans_header(&buf[i..]) {
                        Ok((consumed, msg_option)) => {
                            trace!(
                                "decode_ans_header consumed {}, returned message: {}",
                                consumed,
                                msg_option.is_some()
                            );
                            i += consumed;
                            if msg_option.is_some() {
                                trace!("Complete message decoded in header state, returning");
                                return Ok((i, msg_option)); // Return immediately if message complete
                            }
                        }
                        Err(e) => {
                            error!("Error in decode_ans_header: {:?}", e);
                            self.reset_decoder(); // Reset on error
                            return Err(e);
                        }
                    }
                }
                DecodeStatus::ReceiveResponse => {
                    match self.decode_response(&buf[i..]) {
                        Ok((consumed, msg_option)) => {
                            trace!(
                                "decode_response consumed {}, returned message: {}",
                                consumed,
                                msg_option.is_some()
                            );
                            i += consumed;
                            if msg_option.is_some() {
                                trace!("Complete message decoded in response state, returning");
                                return Ok((i, msg_option)); // Return immediately if message complete
                            }
                        }
                        Err(e) => {
                            error!("Error in decode_response: {:?}", e);
                            self.reset_decoder(); // Reset on error
                            return Err(e);
                        }
                    }
                }
            }
            // Check if state changed, log if it did (useful for debugging loops)
            if self.status != initial_state_for_log {
                trace!("Decoder state changed to: {:?}", self.status);
            }
        }

        trace!(
            "decode finished processing buffer, consumed total {}, returning None",
            i
        );
        Ok((i, None)) // Reached end of buffer without a complete message
    }

    /// Resets the decoder's internal state, typically after an error or to start fresh.
    fn reset_decoder(&mut self) {
        trace!("Resetting decoder state");
        self.start_wait_sync_bytes(0);
    }
}

impl ProtocolEncoder for RplidarHostProtocol {
    /// Encodes a command `Message` into the provided byte buffer.
    ///
    /// Handles adding sync bytes, command flags, payload length, payload data, and checksum.
    fn encode(&mut self, msg: &Message, bytes: &mut [u8]) -> Result<usize> {
        trace!(
            "Encoding message: cmd={:02X}, data_len={}",
            msg.cmd,
            msg.data.len()
        );
        let estimated_encoded_size = self.estimate_encoded_size(msg)?;
        trace!("Estimated encoded size: {}", estimated_encoded_size);

        if estimated_encoded_size > bytes.len() {
            error!(
                "Buffer too small: required {}, available {}",
                estimated_encoded_size,
                bytes.len()
            );
            return Err(Error::BufferTooSmall);
        }

        if msg.data.len() > 255 {
            error!("Payload too large: {} bytes (max 255)", msg.data.len());
            return Err(Error::OperationFail {
                description: "payload too big".to_owned(),
            });
        }

        let has_payload = !msg.data.is_empty();
        let cmd_byte = if has_payload {
            msg.cmd | RPLIDAR_CMDFLAG_HAS_PAYLOAD
        } else {
            msg.cmd
        };
        trace!(
            "Using command byte: {:02X} (has_payload={})",
            cmd_byte,
            has_payload
        );

        bytes[0] = RPLIDAR_CMD_SYNC_BYTE;
        bytes[1] = cmd_byte;

        if has_payload {
            let payload_len = msg.data.len() as u8;
            bytes[2] = payload_len;
            trace!("Payload length byte: {:02X}", payload_len);
            bytes[3..3 + msg.data.len()].clone_from_slice(&msg.data);
            trace!("Copied payload data: {:?}", &bytes[3..3 + msg.data.len()]);

            let mut checksum = Checksum::new();
            checksum.push_slice(&bytes[0..3 + msg.data.len()]); // Checksum includes sync, cmd, len, and data
            let calculated_checksum = checksum.checksum();
            bytes[3 + msg.data.len()] = calculated_checksum;
            trace!("Calculated checksum: {:02X}", calculated_checksum);

            let total_len = 4 + msg.data.len();
            trace!("Total encoded length (with payload): {}", total_len);
            Ok(total_len)
        } else {
            let total_len = 2;
            trace!("Total encoded length (no payload): {}", total_len);
            Ok(total_len)
        }
    }

    /// Estimates the size needed to encode a command `Message`.
    fn estimate_encoded_size(&mut self, msg: &Message) -> Result<usize> {
        // Logging done in encode
        if msg.data.len() > 255 {
            return Err(Error::OperationFail {
                description: "payload too big".to_owned(),
            });
        }

        if !msg.data.is_empty() {
            Ok(4 + msg.data.len()) // Sync + Cmd + Len + Data + Checksum
        } else {
            Ok(2) // Sync + Cmd
        }
    }

    /// Encodes a command `Message` and writes it directly to a `Write` target.
    fn write_to(&mut self, msg: &Message, dest: &mut impl Write) -> Result<usize> {
        trace!(
            "write_to called for message: cmd={:02X}, data_len={}",
            msg.cmd,
            msg.data.len()
        );
        let estimated_encoded_size = self.estimate_encoded_size(msg)?;
        trace!("Allocating buffer of size {}", estimated_encoded_size);
        let mut buf = vec![0; estimated_encoded_size];
        let encoded_size = self.encode(msg, &mut buf[0..estimated_encoded_size])?;
        trace!(
            "Encoded {} bytes into buffer: {:?}",
            encoded_size,
            &buf[0..encoded_size]
        );
        trace!("Writing {} bytes to destination stream...", encoded_size);
        match dest.write_all(&buf[0..encoded_size]) {
            Ok(()) => {
                trace!("Successfully wrote {} bytes", encoded_size); // Add argument
                Ok(encoded_size)
            }
            Err(err) => {
                error!("IO error during write_all: {}", err);
                Err(err.into()) // Keep .into() here as From<io::Error> is implemented
            }
        }
    }

    /// Resets the encoder's internal state (currently a no-op for this protocol).
    fn reset_encoder(&mut self) {
        trace!("Resetting encoder state (no-op)");
        // No state to reset for this encoder
    }
}

#[cfg(test)]
mod tests {

    use crate::base::{Message, ProtocolEncoder, Result};

    fn encode<T: ProtocolEncoder>(protocol: &mut T, msg: &Message) -> Result<Vec<u8>> {
        let encoded_bytes = protocol.estimate_encoded_size(&msg)?;
        let mut buf = vec![0; encoded_bytes];
        let encoded_bytes = protocol.encode(&msg, &mut buf[0..encoded_bytes])?;
        buf.truncate(encoded_bytes);
        Ok(buf)
    }

    #[test]
    fn protocol_encode() {
        let mut protocol = crate::protocol::RplidarHostProtocol::new();

        assert_eq!(
            encode(&mut protocol, &Message::new(0x25))
                .unwrap()
                .as_slice(),
            [0xA5, 0x25]
        );

        assert_eq!(
            encode(&mut protocol, &Message::with_data(0x82, &[0; 5]))
                .unwrap()
                .as_slice(),
            [0xA5, 0x82, 0x05, 0, 0, 0, 0, 0, 0x22]
        );
    }
}
