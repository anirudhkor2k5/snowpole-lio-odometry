# Coordinate frames

## Frames that are actually identified

| Frame | Where it appears | What is known |
| --- | --- | --- |
| WGS 84 geographic, EPSG:4326 | `latitude`, `longitude` in every CSV | Degrees. Used by the existing pipeline. |
| UTM zone 33N, EPSG:32633 | pipeline transformers; pole file; stored `easting`/`northing` when they agree with lat/lon | Metres. False easting 500 km, central meridian 15° E. Confirmed by reprojecting the incremental file with pyproj: residual 0 on the first sample, and by all 290 poles lying on the GNSS track. |
| Dual-GNSS midpoint | constructed, not stored | Mean of the two projected antenna positions. This is the reference track. |
| Antenna baseline | constructed | Left minus right in UTM. Median length 0.499 m, direction about 90° from course. |
| Exported odometry altitude | `altitude` on the LIO CSVs | Not ellipsoidal height. Faster-LIO runs from about 0 m to 2387 m while GNSS altitude stays between 203 m and 387 m. |
| Incremental altitude | `incremental_navigation_results.csv` | Matches GNSS altitude, median difference 0.016 m. This file’s altitude is geographic. |
| Body / LiDAR / IMU | not in the CSVs | The pipeline arrays `gnss_offset` and `lidar_offset` are undocumented in axis order. They were not applied. |
| `rotation_angle_degrees` | odometry CSVs | Not validated as yaw. See below. |

The site longitude is about 9.50° E, which falls in UTM zone 32, not zone 33. The repository and the pole map nevertheless use zone 33. The new code follows that declared frame. It does not switch zones per sample.

## Position representations inside one CSV

For each odometry file the loader keeps two planar positions:

1. `stored_en`: the `easting` and `northing` columns.
2. `wgs84_utm33`: latitude/longitude reprojected to EPSG:32633.

They are the same trajectory only when the disagreement is at the projection tolerance. That is true for `incremental_navigation_results.csv` and false for the three algorithm exports. Metrics are reported for both. Neither column is silently preferred.

`fastreg.csv` lat/lon is a copy of `fastlio.csv` lat/lon. Treating it as a second geographic trajectory would double-count FAST-LIO.

## Orientation

No export contains a validated quaternion or rotation matrix.

Course over ground is computed from consecutive east/north positions, degrees clockwise from north, and is named course. It is undefined on the first sample and noisy when the vehicle is slow.

A heading is also derived from the antenna baseline. The sign (±90° from the baseline azimuth) is chosen so that the heading agrees with GNSS course on samples faster than 2 m/s. That heading is a property of the reference track. It is not an odometry yaw. Median absolute course error against it is about 9.8° for the Faster-LIO lat/lon trajectory and about 21° for the incremental file. Those numbers describe travel direction, not the unused rotation column.

SE(3) relative pose error is not computed. The rotation data required for it failed validation. Relative error is the Euclidean difference of east/north displacements.

## Time

| Stream | Clock | How it is placed on the GNSS clock |
| --- | --- | --- |
| `gps_left_position.csv` | Unix | Reference clock. |
| `gps_right_position.csv` | Unix | Linear interpolation. Median offset −55 µs. |
| `incremental_navigation_results.csv` | Unix | Already the GNSS stamps, max absolute difference 0.24 µs. |
| `fasterlio.csv` | Unix | Used as stored. Starts 0.345 s before the first left-GNSS stamp and ends 0.658 s before the last. |
| `fastlio.csv`, `fastreg.csv` | relative, t = 0 at the first row | Primary origin: `t_unix = t_rel - t_rel[0] + gps_left.t0`. This origin is declared. It is not fit to minimise position error. |

A sensitivity origin, `fasterlio.t0` instead of `gps_left.t0`, moves the relative files by 0.345 s. Absolute error changes by a few metres. It does not create or remove the hundred-metre and kilometre errors.

Resampling onto the reference clock is linear in easting and northing. A reference stamp is kept when it lies inside the odometry span and the bracketing odometry samples are at most 0.15 s apart. Poses are not extrapolated. About 5405 to 5422 of the reference stamps remain, depending on the file.

## Alignments that are and are not used

Reported absolute errors use three planar conventions, all rigid and time-invariant:

1. **Identity.** Both positions stay in EPSG:32633. This is the primary absolute number. The trajectories already start within 0.25 m, so a datum shift is not the discrepancy.
2. **Origin only.** The first synchronised odometry point is translated onto the reference point. No rotation. This does not change the errors, because the start offset is already under 0.25 m.
3. **Initial chord.** The first 30 m of each polyline, and no later motion, supplies one clockwise yaw. The rotated trajectory is then translated so the starts coincide. Yaw estimates are a few degrees to about 15°. Applying them does not reduce absolute error.

A single SE(2) fit on the whole drive, without scale, is stored as `diagnostic_global_se2`. It is not an accuracy result. Fitting the test trajectory would remove a constant frame rotation and hide part of the error the experiment is supposed to measure. Even that diagnostic leaves 369 m RMSE on the Faster-LIO lat/lon trajectory and 54 m RMSE on the incremental file, so the mismatch is not only a fixed yaw.

No time-varying alignment, no scale, and no ground-truth pose is used as an input to any runtime feature.

## What is unresolved

- Which Faster-LIO column is the pose the black box actually produced.
- Why the exported LIO altitude integrates to kilometres.
- Body-to-LiDAR, body-to-IMU, and body-to-antenna rotations. The 0.50 m antenna baseline is measured. The pipeline’s 3-vectors are not a substitute.
- Whether `gps_state` or `utm_position` in the bag would change the fix-type interpretation. Those messages were not decoded.
- Per-point time offsets inside each Ouster scan. The image topics exist; the point cloud exists only in the full bag; neither was decoded.
