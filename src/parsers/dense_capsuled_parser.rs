//! Parser logic for RPLIDAR dense capsuled measurement responses (0x85).

use crate::answers::{
    RplidarResponseDenseCapsuleMeasurementNodes, RplidarResponseMeasurementNodeHq,
    RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT,
};
use crate::internals::CachedPrevCapsule;
use crate::parsers::capsuled_parser::{
    angle_diff_q8, angle_q6_to_angle_z_q14, generate_flag, ANGLE_360_Q16,
};
use log::trace;
use log::warn;

// Number of points (cabins) in a dense capsule
const DENSE_CABIN_COUNT: usize = 40;

/// Extracts the start angle (Q8 format) from a dense capsule.
#[inline]
fn get_start_angle_q8(nodes: &RplidarResponseDenseCapsuleMeasurementNodes) -> u32 {
    ((nodes.start_angle_sync_q6 & 0x7fffu16) as u32) << 2
}

/// Checks if the current angle represents a sync point based on the increment.
/// This version includes the logic from the C++ code involving `last_node_sync_bit`.
#[inline]
fn check_dense_sync(cur_angle_q16: u32, angle_inc_q16: u32, last_sync_bit: &mut u8) -> bool {
    let mut sync_bit = if ((cur_angle_q16 + angle_inc_q16) % ANGLE_360_Q16) < (angle_inc_q16 << 1) {
        1
    } else {
        0
    };
    sync_bit = (sync_bit ^ *last_sync_bit) & sync_bit;
    *last_sync_bit = if sync_bit != 0 { 1 } else { 0 };
    sync_bit != 0
}

/// Converts a dense cabin node and angle info into a standard HQ node.
#[inline]
fn to_hq(
    dist_q2: u32,
    cur_angle_raw_q16: u32,
    angle_inc_q16: u32,
    last_sync_bit: &mut u8,
) -> RplidarResponseMeasurementNodeHq {
    let angle_q6 = cur_angle_raw_q16 >> 10;
    let sync = check_dense_sync(cur_angle_raw_q16, angle_inc_q16, last_sync_bit);

    RplidarResponseMeasurementNodeHq {
        angle_z_q14: angle_q6_to_angle_z_q14(angle_q6),
        dist_mm_q2: dist_q2,
        quality: if dist_q2 != 0 {
            0x2F << RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT
        } else {
            0
        },
        flag: generate_flag(sync),
    }
}

/// Parses a `RplidarResponseDenseCapsuleMeasurementNodes` response packet.
///
/// Requires the previously received dense capsule (`cached_prev`) to calculate angular increments.
/// Returns a vector of `RplidarResponseMeasurementNodeHq` points derived from the *previous* dense capsule,
/// and the current dense capsule to be cached for the next iteration.
///
/// # Arguments
///
/// * `cached_prev` - The `CachedPrevCapsule` containing the data from the immediately preceding dense capsuled response packet.
/// * `nodes` - The current `RplidarResponseDenseCapsuleMeasurementNodes` packet to parse.
pub fn parse_dense_capsuled(
    cached_prev: &CachedPrevCapsule,
    nodes: RplidarResponseDenseCapsuleMeasurementNodes,
) -> (Vec<RplidarResponseMeasurementNodeHq>, CachedPrevCapsule) {
    let start_angle_sync_q6 = nodes.start_angle_sync_q6;
    trace!(
        "Parsing dense capsule: start_angle_q6={:04X}",
        start_angle_sync_q6
    );
    if let CachedPrevCapsule::DenseCapsuled(prev_capsule) = cached_prev {
        let prev_start_angle_sync_q6 = prev_capsule.start_angle_sync_q6;
        trace!(
            "Using previous dense capsule: start_angle_q6={:04X}",
            prev_start_angle_sync_q6
        );
        let mut output_nodes: Vec<RplidarResponseMeasurementNodeHq> = Vec::with_capacity(40);

        let cur_start_angle_q8 = get_start_angle_q8(&nodes);
        let prev_start_angle_q8 = get_start_angle_q8(prev_capsule);
        trace!(
            "Current start angle Q8: {}, Previous start angle Q8: {}",
            cur_start_angle_q8,
            prev_start_angle_q8
        );

        let diff_angle_q8 = angle_diff_q8(prev_start_angle_q8, cur_start_angle_q8);
        trace!("Difference angle Q8: {}", diff_angle_q8);

        let angle_inc_q16 = (diff_angle_q8 << 8) / (DENSE_CABIN_COUNT as u32);
        let mut cur_angle_raw_q16 = prev_start_angle_q8 << 8;
        let mut last_sync_bit: u8 = 0;
        trace!(
            "Angle increment Q16 (per node): {}, Initial angle Q16: {}",
            angle_inc_q16,
            cur_angle_raw_q16
        );

        for cabin in prev_capsule.cabins.iter() {
            let dist_q2 = (cabin.distance as u32) << 2;

            output_nodes.push(to_hq(
                dist_q2,
                cur_angle_raw_q16,
                angle_inc_q16,
                &mut last_sync_bit,
            ));
            cur_angle_raw_q16 += angle_inc_q16;
        }
        trace!(
            "Finished parsing dense capsule, generated {} nodes",
            output_nodes.len()
        );
        (output_nodes, CachedPrevCapsule::DenseCapsuled(nodes))
    } else {
        warn!("Previous capsule was not 'DenseCapsuled' (was {:?}), cannot calculate angles for current capsule. Caching current and returning empty.", cached_prev);
        (Vec::new(), CachedPrevCapsule::DenseCapsuled(nodes))
    }
}
