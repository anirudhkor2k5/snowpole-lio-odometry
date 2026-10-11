# Data validation

## Checks that passed

- CSV timestamps are strictly increasing. No duplicate stamps and no step above 0.5 s on the pose or GNSS files.
- Header sequence numbers increase by 1 inside each file.
- `incremental_navigation_results.csv` stamps match `gps_left_position.csv` to 0.24 µs, and its lat/lon matches its stored UTM to pyproj’s tolerance.
- Reprojecting the GNSS track to EPSG:32633 puts every mapped pole within 15 m of the track (median 5.53 m). The zone used by the pipeline is the zone of this recording.
- The two antennas stay 0.499 m apart (std 0.013 m) and that baseline stays lateral to the direction of travel.
- Source SHA-256 values were unchanged by the audit and the baseline script.
- Bag indexes open as ROS bag v2 and the GNSS message counts match the CSV row counts (5422).

## Checks that failed or stay unresolved

### Serious: two positions in the Faster-LIO file

`fasterlio.csv` latitude/longitude and stored easting/northing differ by a median of 271 m. Their 1 s relative motion also differs by a median of 1.59 m, so they are not one trajectory written two ways. Both start within 0.1 m of the GNSS midpoint and then diverge from GNSS and from each other. Absolute evaluation without saying which column was used would be arbitrary. Both are reported. Neither is accepted as the pose.

`fastlio.csv` is milder and still too large for a metre-level claim: median 10.1 m between the columns, 0.16 m of relative motion over 1 s. `fastreg.csv` lat/lon is a copy of `fastlio.csv`; its stored easting/northing is a different path (median 313 m from the copied lat/lon).

### Serious: LIO altitude is not height

Faster-LIO altitude runs from about 0 m to 2387 m. FAST-LIO and the FastReg export share an altitude that runs to 2974 m. GNSS altitude on the same drive stays inside 203–387 m and descends overall. Horizontal metrics exclude this channel. It is reported only as a property of the export.

The incremental file’s altitude does match GNSS.

### `rotation_angle_degrees` is not yaw

See `docs/02_coordinate_frames.md`. Using it as an orientation would fabricate SE(3) poses.

### The reference is GNSS, not ground truth

There is no independent post-processed trajectory in the checkout. Dual GNSS is the reference the existing paper also uses, with the limitations above: unknown covariance type, zero reported up-variance, service bits unset, and no decoded `gps_state`. Absolute numbers in `docs/04_baseline_results.md` are GNSS-referenced errors. They are not survey accuracy.

Common-mode GNSS bias cancels in short-horizon differences only to the extent that the bias is constant over that horizon. The centimetre antenna baseline says the two receivers agree with each other. It does not put a centimetre bound on that common mode.

### One drive

One recording, about 542 s and 10.3 km. There is no second journey, no weather split, and no geographic holdout. Chronological blocks are the only split that avoids shuffling adjacent frames. The error grows along the drive, so the last block is a different operating region from the first.

### Sensor streams required for geometry were not decoded

IMU (54202 messages), Ouster images (5420 each), `utm_position`, `gps_state`, `/ouster/points` (5419, full bag only), and `/vehicle_enu_heading` (10833, full bag only) are present in the index. Reading them means decompressing chunks that also contain images or the 41 GB bag’s other topics. That was not done. No IMU feature, LiDAR geometric feature, or weather proxy is claimed.

`Trip068.json`, the Ouster metadata the existing pipeline uses to turn a range image into XYZ, is not in the checkout. Beam angles were not guessed.

### Empty pose covariance

Faster-LIO, FAST-LIO, and `fastreg.csv` have empty pose-covariance columns. There is no backend covariance to calibrate against.

## What this blocks

Supervised claims that a number is the vehicle’s error against survey ground truth.

A single Faster-LIO trajectory for training, until the lat/lon and easting/northing columns are reconciled from the original estimator output.

IMU and scan features, until the bag payloads and the Ouster metadata are read.

SE(3) metrics, until a real orientation exists.

Drop-in replacement of `incremental_navigation_results.csv` by `fasterlio.csv` inside the current pipeline. The row counts differ, and the pipeline aligns by index.

## What remains valid

Identity-frame horizontal error and relative displacement error of each exported column against the dual-GNSS midpoint, with the reference named as GNSS.

Causal motion features computed from those columns, with labels stored in a separate file.

A negative reliability result on this one drive, which is what `docs/05_reliability_experiment.md` reports.
