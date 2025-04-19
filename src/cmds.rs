// Commands without payload and response

/// Command code to stop the measurement process of the LIDAR.
pub const RPLIDAR_CMD_STOP: u8 = 0x25;

/// Command code to start a scan in the default mode (typically legacy scan mode).
pub const RPLIDAR_CMD_SCAN: u8 = 0x20;

/// Command code to start a forced scan in the default mode.
/// A forced scan attempts to take measurements even if the motor is not spinning at the correct speed.
pub const RPLIDAR_CMD_FORCE_SCAN: u8 = 0x21;

/// Command code to reset the LIDAR core. Requires re-initialization afterwards.
pub const RPLIDAR_CMD_RESET: u8 = 0x40;

// Commands without payload but have response

/// Command code to request device information (model, firmware, hardware, serial number).
pub const RPLIDAR_CMD_GET_DEVICE_INFO: u8 = 0x50;

/// Command code to request the device's health status.
pub const RPLIDAR_CMD_GET_DEVICE_HEALTH: u8 = 0x52;

// Commands with payload and have response

/// Command code to start an express scan. Used for various scan modes beyond the legacy mode.
/// Requires a `RplidarPayloadExpressScan` payload. Added in firmware version 1.17.
pub const RPLIDAR_CMD_EXPRESS_SCAN: u8 = 0x82;

/// Payload structure for the `RPLIDAR_CMD_EXPRESS_SCAN` command.
#[derive(Debug, Copy, Clone, PartialEq)] // Added derive for better usability
#[repr(C, packed)]
pub struct RplidarPayloadExpressScan {
    /// The requested working mode ID. 0 typically refers to a legacy express mode.
    /// Other values correspond to specific scan mode IDs obtained via `get_all_supported_scan_modes`.
    pub work_mode: u8,

    /// Reserved flags. Should be set to 0.
    pub work_flags: u16,

    /// Reserved parameter. Should be set to 0.
    pub param: u16,
}

/// Command code to retrieve LIDAR configuration parameters. Added in firmware version 1.24.
/// Requires a payload specifying the configuration type ID (e.g., `RPLIDAR_CONF_SCAN_MODE_TYPICAL`).
pub const RPLIDAR_CMD_GET_LIDAR_CONF: u8 = 0x84;

/// Command code to set the motor PWM duty cycle via an accessory board (e.g., for A2/A3 models).
/// Requires a 2-byte payload containing the PWM value (u16 little-endian).
pub const RPLIDAR_CMD_SET_MOTOR_PWM: u8 = 0xF0;

/// Command code to get the capability flags of an attached accessory board.
/// Requires a 4-byte payload (usually all zeros).
pub const RPLIDAR_CMD_GET_ACC_BOARD_FLAG: u8 = 0xFF;

// LIDAR configurations (Used as payload data for RPLIDAR_CMD_GET_LIDAR_CONF)

/// Configuration type ID to request the total number of supported scan modes. Response is u16.
pub const RPLIDAR_CONF_SCAN_MODE_COUNT: u32 = 0x00000070;

/// Configuration type ID to request the sample duration (in microseconds, Q8.16 format) for a specific scan mode.
/// Requires a 2-byte payload (u16 scan mode ID). Response is u32.
pub const RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE: u32 = 0x00000071;

/// Configuration type ID to request the maximum measurable distance (in millimeters, Q8.16 format) for a specific scan mode.
/// Requires a 2-byte payload (u16 scan mode ID). Response is u32.
pub const RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE: u32 = 0x00000074;

/// Configuration type ID to request the answer type (response command code) used for measurements in a specific scan mode.
/// Requires a 2-byte payload (u16 scan mode ID). Response is u8.
pub const RPLIDAR_CONF_SCAN_MODE_ANS_TYPE: u32 = 0x00000075;

/// Configuration type ID to request the ID of the device's typical or default scan mode. Response is u16.
pub const RPLIDAR_CONF_SCAN_MODE_TYPICAL: u32 = 0x0000007C;

/// Configuration type ID to request the name (as a string) of a specific scan mode.
/// Requires a 2-byte payload (u16 scan mode ID). Response is a null-terminated string (up to 64 bytes).
pub const RPLIDAR_CONF_SCAN_MODE_NAME: u32 = 0x0000007F;
