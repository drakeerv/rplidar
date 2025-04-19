use crate::types::ScanPoint;
use log::trace;
use std::f32::consts::TAU;

/// Sorts a slice of `ScanPoint`s by angle and attempts to correct angle inconsistencies.
///
/// This function first sorts the points by their raw angle values. Then, it identifies
/// the longest contiguous span of valid points (quality > 0 and distance > 0).
/// It calculates the average angular increment between consecutive valid points within this span.
/// This average increment is then used to linearly interpolate/extrapolate angles for all points,
/// anchored to the first valid point's angle, wrapping around 2*PI (TAU) as needed.
/// Finally, it re-sorts the points based on the corrected angles.
///
/// This helps to smooth out minor angular errors and handle the angle wrap-around
/// inherent in LIDAR scans more robustly.
///
/// # Arguments
///
/// * `scan` - A mutable slice of `ScanPoint`s to be sorted and corrected.
///
/// # Returns
///
/// * `Result<()>` - Returns `Ok(())` on success. Errors are currently not returned but might be added in the future.
pub fn sort_scan(scan: &mut [ScanPoint]) -> crate::base::Result<()> {
    trace!("sort_scan called with {} points", scan.len());

    if scan.is_empty() {
        trace!("Scan is empty, nothing to sort");
        return Ok(());
    }

    // Initial sort by raw angle
    trace!("Sorting scan by angle_z_q14 initially...");
    scan.sort_unstable();

    // Find the first and last valid indices
    let first_valid_idx = scan.iter().position(|p| p.is_valid());
    let last_valid_idx = scan.iter().rposition(|p| p.is_valid());
    trace!(
        "First valid index: {:?}, Last valid index: {:?}",
        first_valid_idx,
        last_valid_idx
    );

    if let (Some(first_idx), Some(last_idx)) = (first_valid_idx, last_valid_idx) {
        if first_idx >= last_idx {
            trace!("No valid span found or only one valid point (first_idx={}, last_idx={}), skipping angle correction", first_idx, last_idx);
        } else {
            // Calculate average increment across the valid span [first_idx, last_idx]
            let mut total_increment = 0.0;
            let mut num_valid_increments = 0;

            for i in first_idx..last_idx {
                // Iterate up to last_idx - 1
                // Check if both current and next points are valid for increment calculation
                if scan[i].is_valid() && scan[i + 1].is_valid() {
                    let angle_i = scan[i].angle();
                    let angle_i_plus_1 = scan[i + 1].angle();

                    let delta_angle = if angle_i_plus_1 >= angle_i {
                        angle_i_plus_1 - angle_i
                    } else {
                        // Handle wrap-around
                        TAU - angle_i + angle_i_plus_1
                    };

                    // Filter out unreasonably large increments which might indicate bad data or missed points
                    // A typical increment is small. Cap it at TAU / 10 (~36 degrees).
                    const MAX_REASONABLE_INCREMENT: f32 = TAU / 10.0;
                    if delta_angle > 1e-6 && delta_angle < MAX_REASONABLE_INCREMENT {
                        // Check > small epsilon
                        total_increment += delta_angle;
                        num_valid_increments += 1;
                    } else {
                        trace!(
                            "Skipping unreasonable increment between index {} and {}: {:.4} rad",
                            i,
                            i + 1,
                            delta_angle
                        );
                    }
                }
            }

            if num_valid_increments > 0 {
                let avg_increment = total_increment / num_valid_increments as f32;
                trace!(
                    "Calculated average increment over {} valid steps: {:.6} rad",
                    num_valid_increments,
                    avg_increment
                );

                if avg_increment > 1e-6 {
                    // Ensure increment is positive and non-trivial
                    // Use the angle of the first valid point as the anchor
                    let anchor_angle = scan[first_idx].angle();
                    trace!(
                        "Using anchor angle at index {}: {:.4} rad",
                        first_idx,
                        anchor_angle
                    );

                    // Correct/Extrapolate angles *before* the anchor index
                    trace!("Correcting/Extrapolating angles before index {}", first_idx);
                    for i in (0..first_idx).rev() {
                        let mut current_angle =
                            anchor_angle - ((first_idx - i) as f32 * avg_increment);
                        // Handle wrap-around (normalize to 0..TAU)
                        current_angle = current_angle.rem_euclid(TAU);
                        scan[i].set_angle(current_angle);
                        // trace!("  Corrected index {}: new angle {:.4} rad", i, current_angle);
                    }

                    // Set the anchor point's angle (might be redundant, but ensures consistency)
                    scan[first_idx].set_angle(anchor_angle);

                    // Correct/Extrapolate angles *after* the anchor index
                    trace!("Correcting/Extrapolating angles after index {}", first_idx);
                    // Note: Using enumerate().skip() as suggested by clippy.
                    // The index `i` here is the original index in the slice.
                    for (i, point) in scan.iter_mut().enumerate().skip(first_idx + 1) {
                        let mut current_angle =
                            anchor_angle + ((i - first_idx) as f32 * avg_increment);
                        // Handle wrap-around (normalize to 0..TAU)
                        current_angle = current_angle.rem_euclid(TAU);
                        point.set_angle(current_angle);
                        // trace!("  Corrected index {}: new angle {:.4} rad", i, current_angle);
                    }

                    // Re-sort based on corrected angles
                    trace!("Re-sorting scan after angle correction...");
                    scan.sort_unstable();
                } else {
                    trace!("Average increment is too small or negative ({:.6}), skipping angle correction", avg_increment);
                }
            } else {
                trace!("No valid increments found between indices {} and {}, skipping angle correction", first_idx, last_idx);
            }
        }
    } else {
        trace!("No valid points found in scan, cannot correct angles");
    }

    trace!("sort_scan finished successfully");
    Ok(())
}
