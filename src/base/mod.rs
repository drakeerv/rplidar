//! # RPOS Driver Infrastructure
//!
//! `rpos_drv` is a collection of structs and traits to build drivers for RPOS.

mod channel;
// mod prelude; // Removed
mod error;
mod message; // Added
mod ring_byte_buffer;
mod traits; // Added

// Re-export common driver items.
pub use self::error::{Error, Result}; // Re-export Error and Result
pub use self::message::Message; // Re-export Message
pub use self::traits::{ProtocolDecoder, ProtocolEncoder}; // Re-export traits
                                                          // Re-export the Channel struct for communication.
pub use self::channel::*;
// Re-export the RingByteBuffer utility.
pub use self::ring_byte_buffer::RingByteBuffer;
