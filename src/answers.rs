/// Response type identifier for device information.
pub const RPLIDAR_ANS_TYPE_DEVINFO: u8 = 0x4;

/// Data structure containing device information received from the RPLIDAR.
#[derive(Debug, Copy, Clone, PartialEq)]
#[repr(C, packed)]
pub struct RplidarResponseDeviceInfo {
    /// Model ID of the RPLIDAR.
    pub model: u8,
    /// Firmware version (major << 8 | minor).
    pub firmware_version: u16,
    /// Hardware version.
    pub hardware_version: u8,
    /// 16-byte unique serial number.
    pub serialnum: [u8; 16],
}

/// Response type identifier for device health status.
pub const RPLIDAR_ANS_TYPE_DEVHEALTH: u8 = 0x6;

/// Data structure containing device health status received from the RPLIDAR.
#[derive(Debug, Copy, Clone, PartialEq)]
#[repr(C, packed)]
pub struct RplidarResponseDeviceHealth {
    /// Health status code (see `RPLIDAR_HEALTH_STATUS_*` constants).
    pub status: u8,
    /// Error code associated with the status (if status is Warning or Error).
    pub error_code: u16,
}

// health status

/// Health status code indicating the LIDAR is operating correctly.
pub const RPLIDAR_HEALTH_STATUS_OK: u8 = 0;

/// Health status code indicating a non-critical warning. The LIDAR might still function.
pub const RPLIDAR_HEALTH_STATUS_WARNING: u8 = 1;

/// Health status code indicating a critical error. The LIDAR is likely non-operational.
pub const RPLIDAR_HEALTH_STATUS_ERROR: u8 = 2;

// Measurement answers

/// Response type identifier for legacy measurement data (single point per response).
pub const RPLIDAR_ANS_TYPE_MEASUREMENT: u8 = 0x81;

/// Data structure for a single legacy measurement point.
/// Max distance: 16.384 meters.
#[derive(Debug, Copy, Clone, PartialEq)]
#[repr(C, packed)]
pub struct RplidarResponseMeasurementNode {
    /// Contains sync bit (LSB) and quality (upper 6 bits).
    pub sync_quality: u8,
    /// Contains angle (Q6 format, upper 15 bits) and check bit (LSB).
    pub angle_q6_checkbit: u16,
    /// Distance measurement in Q2 format (millimeters).
    pub distance_q2: u16,
}

/// Mask for extracting the sync bit from `sync_quality` in legacy measurement nodes.
pub const RPLIDAR_RESP_MEASUREMENT_SYNCBIT: u8 = 1;
/// Bit shift for extracting the quality value from `sync_quality` in legacy measurement nodes.
pub const RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT: usize = 2;
/// Bit shift for extracting the angle value from `angle_q6_checkbit` in legacy measurement nodes.
pub const RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT: usize = 1;

/// Response type identifier for capsuled measurement data (multiple points per response).
/// Added in firmware version 1.17.
pub const RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED: u8 = 0x82;

/// Data structure representing a "cabin" containing two measurement points within a capsuled response.
#[derive(Debug, Copy, Clone, PartialEq)]
#[repr(C, packed)]
pub struct RplidarResponseCabinNodes {
    /// Combined distance (Q2) and partial angle offset for the first point.
    pub distance_angle_1: u16,
    /// Combined distance (Q2) and partial angle offset for the second point.
    pub distance_angle_2: u16,
    /// Combined partial angle offsets (Q3) for both points.
    pub offset_angles_q3: u8,
}

/// Data structure for a complete capsuled measurement response packet (contains 16 cabins, 32 points).
#[derive(Debug, Copy, Clone, PartialEq)]
#[repr(C, packed)]
pub struct RplidarResponseCapsuleMeasurementNodes {
    /// First sync/checksum byte.
    pub s_checksum_1: u8,
    /// Second sync/checksum byte.
    pub s_checksum_2: u8,
    /// Start angle of the first point in the capsule (Q6 format).
    pub start_angle_sync_q6: u16,
    /// Array of 16 cabins.
    pub cabins: [RplidarResponseCabinNodes; 16],
}

/// Expected value of the upper nibble of the first sync/checksum byte in capsuled responses.
pub const RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_1: u8 = 0xA;
/// Expected value of the upper nibble of the second sync/checksum byte in capsuled responses.
pub const RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_2: u8 = 0x5;

/// Response type identifier for high-quality measurement data (single point, higher precision).
pub const RPLIDAR_ANS_TYPE_MEASUREMENT_HQ: u8 = 0x83;

/// Data structure for a single high-quality measurement point.
#[derive(Debug, Copy, Clone, PartialEq)]
#[repr(C, packed)]
pub struct RplidarResponseMeasurementNodeHq {
    /// Angle measurement (Z-angle) in Q14 format.
    pub angle_z_q14: u16,
    /// Distance measurement in Q2 format (millimeters).
    pub dist_mm_q2: u32,
    /// Quality indicator.
    pub quality: u8,
    /// Flag, LSB indicates sync point.
    pub flag: u8,
}

/// Data structure for a high-quality capsuled measurement response packet (contains 16 HQ points).
#[derive(Debug, Copy, Clone, PartialEq)]
#[repr(C, packed)]
pub struct RplidarResponseHqCapsuledMeasurementNodes {
    /// Sync byte (should be `RPLIDAR_RESP_MEASUREMENT_HQ_SYNC`).
    pub sync_byte: u8,
    /// Timestamp (microseconds). Meaning might vary by device/firmware.
    pub timestamp: u64,
    /// Array of 16 high-quality measurement nodes.
    pub nodes: [RplidarResponseMeasurementNodeHq; 16],
    /// CRC32 checksum of the packet (excluding this field).
    pub crc32: u32,
}

/// Mask for extracting the sync bit from the `flag` field in HQ measurement nodes.
pub const RPLIDAR_RESP_HQ_FLAG_SYNCBIT: u8 = 1;
/// Expected sync byte value for HQ capsuled measurement responses.
pub const RPLIDAR_RESP_MEASUREMENT_HQ_SYNC: u8 = 0xA5;

/// Response type identifier for ultra-capsuled measurement data (more points per response).
/// Added in firmware version 1.23alpha.
pub const RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED_ULTRA: u8 = 0x84;

/// Response type identifier for dense capsuled measurement data.
/// Added in firmware version 1.25.
pub const SL_LIDAR_ANS_TYPE_MEASUREMENT_DENSE_CAPSULED: u8 = 0x85;

/// Data structure representing a single cabin in a dense capsuled response.
#[derive(Debug, Copy, Clone, PartialEq)]
#[repr(C, packed)]
pub struct RplidarResponseDenseCabinNodes {
    /// Distance measurement (format depends on context, likely Q2 or similar after scaling).
    pub distance: u16,
}

/// Data structure for a complete dense capsuled measurement response packet (contains 40 cabins, 40 points).
#[derive(Debug, Copy, Clone, PartialEq)]
#[repr(C, packed)]
pub struct RplidarResponseDenseCapsuleMeasurementNodes {
    /// First sync/checksum byte.
    pub s_checksum_1: u8,
    /// Second sync/checksum byte.
    pub s_checksum_2: u8,
    /// Start angle of the first point in the capsule (Q6 format).
    pub start_angle_sync_q6: u16,
    /// Array of 40 dense cabins.
    pub cabins: [RplidarResponseDenseCabinNodes; 40],
}

/// Data structure for a complete ultra-capsuled measurement response packet (contains 32 ultra cabins, 96 points).
#[derive(Debug, Copy, Clone, PartialEq)]
#[repr(C, packed)]
pub struct RplidarResponseUltraCapsuleMeasurementNodes {
    /// First sync/checksum byte.
    pub s_checksum_1: u8,
    /// Second sync/checksum byte.
    pub s_checksum_2: u8,
    /// Start angle of the first point in the capsule (Q6 format).
    pub start_angle_sync_q6: u16,
    /// Array of 32 ultra cabins (each encodes 3 points).
    pub ultra_cabins: [u32; 32],
}

/// Response type identifier for getting LIDAR configuration parameters.
/// Added in firmware version 1.24.
pub const RPLIDAR_ANS_TYPE_GET_LIDAR_CONF: u8 = 0x20;

/// Response type identifier for getting accessory board capability flags.
pub const RPLIDAR_ANS_TYPE_ACC_BOARD_FLAG: u8 = 0xFF;

/// Bitmask for checking motor control support in the accessory board flags response.
pub const RPLIDAR_RESP_ACC_BOARD_FLAG_MOTOR_CTRL_SUPPORT_MASK: u32 = 0x1;
