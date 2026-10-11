# Data audit

## What was inspected

Read-only inspection of the repository checkout at commit `301e38a9025716e0550c3bb0bc192dc9a8a414af`. No original CSV, bag, or source file was modified. SHA-256 checks before and after the baseline script matched.

| File | Rows | Role |
| --- | ---: | --- |
| `fasterlio.csv` | 5415 | Faster-LIO export, primary research backbone |
| `fastlio.csv` | 5416 | FAST-LIO export |
| `fastreg.csv` | 5416 | Additional FastReg-labelled export |
| `incremental_navigation_results.csv` | 5422 | File the snow-pole pipeline actually reads |
| `gps_left_position.csv` | 5422 | Left GNSS `NavSatFix` export |
| `gps_right_position.csv` | 5422 | Right GNSS `NavSatFix` export |
| `Groundtruth_pole_location_at_test_site_E39_Hemnekjølen.csv` | 290 | Mapped pole coordinates, not a vehicle trajectory |
| `2024-02-28-12-59-51_no_unwanted_topics.bag` | 5.7 GB | Reduced bag, index only |
| `2024-02-28-12-59-51.bag` | 41 GB | Full bag, index only |
| `faster-lio.pdf` | 8 pages | Bai et al., RA-L 2022 |

There is no DLIO file, no GenZ-LIO file, and no separately extracted survey trajectory. Those algorithms were not installed.

Machine-readable counts are in `reports/dataset_statistics.json`.

## How the existing pipeline reads odometry

Both entry points load one CSV and use two columns:

- `snowpole_based_vehicle_localization.py` lines 27–33
- `snowpole_based_vehicle_localization_GNSS_percentage.py` lines 26–31

```text
file_path = incremental_navigation_results.csv
easting, northing
```

`fastreg.csv` is not the pipeline input. The code comments call `incremental_navigation_results.csv` the FastReg result.

### Columns the pipeline requires

Only `easting` and `northing`. Heading and step length are derived afterwards. `rotation_angle_degrees` is not read.

### Coordinate reference system

`EPSG:32633` (WGS 84 / UTM zone 33N) and `EPSG:4326`. See `docs/02_coordinate_frames.md`. The pole file uses the same zone: columns `UTM33-Nord` and `UTM33-Øst`.

### Heading

`snowpole_based_vehicle_localization.py` lines 49–66. Consecutive UTM positions are converted to latitude/longitude and the geodesic forward azimuth is taken with GeographicLib on WGS 84. The result is normalised to `[0, 360)` degrees clockwise from north. The first sample is NaN. This is course over ground, not a body-yaw measurement, and it is not the CSV rotation column.

### Timestamps and LiDAR association

The pipeline does not associate odometry to LiDAR by timestamp. It walks range images by index and consumes `headings[i]` and `fastreg_eastings[i]` at that same index (`snowpole_based_vehicle_localization.py` around lines 171–195). GNSS is read at index `i+1` (lines 109–111). There are 5420 range images in the reduced bag and 5422 GNSS/incremental rows, so the index is already off by design.

`fasterlio.csv` has 5415 rows. It cannot be substituted for `incremental_navigation_results.csv` by row index.

### Sensor-frame transforms in the existing code

Hard-coded in `snowpole_based_vehicle_localization.py` lines 122–124, metres, axis convention not stated beyond the arrays:

```text
gnss_offset  = [-0.32, 0.0, 1.24]
lidar_offset = [ 0.70, 0.0, 1.80]
```

This is one GNSS offset, not a left/right antenna baseline. It was not applied in the new evaluation. Applying an undocumented lever arm would invent a calibration.

### Pole integration

Inside the pole bounding box, YOLOv5s detections on the signal image are back-projected with the Ouster lookup (`Trip068.json`, which is not in this checkout). Detected poles are associated with `Groundtruth_pole_location_at_test_site_E39_Hemnekjølen.csv`. That file is a landmark map.

### Outputs and evaluation already in the repo

The pipeline’s own score is the Euclidean UTM distance between the odometry easting/northing and a kriged average of the two GNSS receivers. That is not ATE/RPE, and the kriging uses the same drive it evaluates. The new baseline does not use that kriging.

## Odometry CSV schema

All three algorithm exports and both GNSS files share a bagpy `sensor_msgs/NavSatFix` header. Odometry files add `easting`, `northing`, `translation_easting`, `translation_northing`, `rotation_angle_degrees`, `transformed_easting`, `transformed_northing`, and `error`.

| Export | Time basis | Duration | Median dt | Covariance, status, translation, error |
| --- | --- | ---: | ---: | --- |
| fasterlio | Unix, equals `header.stamp` | 541.439 s | 0.09999 s | empty on all 5415 rows |
| fastlio | relative, starts at 0 | 541.540 s | ~0.100 s | empty |
| fastreg | relative, identical stamps to fastlio | 541.540 s | ~0.100 s | empty |
| incremental | Unix, matches `gps_left` within 0.24 µs | 541.753 s | ~0.100 s | populated |
| gps left / right | Unix | 541.753 s | ~0.100 s | populated |

Sequence numbers increase by 1 with no gaps. GNSS sequences start at 1166 and 1165, so the export is not from the start of the bag index. Odometry sequences were reindexed from 0.

No non-monotonic timestamps and no gap above 0.5 s. This is a nominal 10 Hz pose stream. The LiDAR image topics also have 5420 messages, consistent with 10 Hz over this drive. IMU in the bag index has 54202 messages, about 100 Hz. None of those IMU samples are in the CSVs.

## Internal column checks

Latitude/longitude were reprojected with pyproj to `EPSG:32633` and compared with the stored easting/northing.

| File | Median horizontal disagreement | 1 s relative disagreement between the two columns |
| --- | ---: | ---: |
| fasterlio | 270.6 m (max 591.9 m) | 1.59 m |
| fastlio | 10.1 m (max 15.9 m) | 0.16 m |
| fastreg | 313.2 m (max 627.7 m) | 1.84 m |
| incremental | 0.0 m (pyproj residual on the first sample is 0) | ~0 |

`fastreg.csv` latitude, longitude, altitude, and `rotation_angle_degrees` match `fastlio.csv` exactly. Stored easting/northing do not: maximum absolute differences are 480 m east and 395 m north. The lat/lon columns of `fastreg.csv` are not an independent FastReg solution.

On `incremental_navigation_results.csv`, `translation_easting` and `translation_northing` from row 1 onward equal the position step exactly (median residual 0). The `error` column is populated (median about 1.4 m in the raw file) but it is not the distance to GNSS. GNSS-referenced horizontal error for this file has median 50.2 m.

## `rotation_angle_degrees`

The column is populated on the odometry exports and is an angle in degrees, roughly in `[-115, 119]`. It is not an incremental yaw: at 10 Hz an incremental yaw would be a few degrees per sample, and the cumulative sum is not a meaningful heading.

After synchronizing each export to the GNSS clock, the median absolute difference between this column and course over ground is 64° to 72°. Correlation with course is about 0.06 for Faster-LIO and 0.45 for the incremental file. Correlation with the dual-antenna heading is no better. The column is not body yaw and is not used as a pose orientation.

## Reference trajectory

The reference used here is the midpoint of the two GNSS solutions after projecting both to UTM 33N and interpolating the right antenna onto the left timestamps. The median left-minus-right stamp offset is −55 µs.

| Quantity | Value |
| --- | ---: |
| Duration | 541.75 s |
| Path length | 10,298 m |
| Start-to-end distance | 8,285 m |
| Antenna separation, median (std) | 0.499 m (0.013 m) |
| Altitude | 203.2 m to 387.2 m, median 373.1 m |
| Samples with supervision weight below 0.5 | 0.74% |

The baseline azimuth stays about 90° off the direction of travel, which is what a left/right roof pair should do. Separation stable to about a centimetre means the two solutions share their horizontal error or are both precise relative to each other. It does not measure absolute accuracy.

`status.status` is 2 on 5382 of 5422 left-receiver samples and 0 on 40. In `sensor_msgs/NavSatStatus`, 2 is `STATUS_GBAS_FIX` and 0 is `STATUS_FIX`. `status.service` is 0 on every sample, so no constellation bits are set. `position_covariance_type` is 0 (`COVARIANCE_TYPE_UNKNOWN`) on every sample. The up-variance is 0 on every sample. Median reported east/north variances on the left receiver are 52 m² and 111 m². If those numbers were real independent variances they would be several metres of noise, which contradicts the centimetre baseline stability. They are kept as a quality flag only. They are not a covariance we can use in a filter.

All 290 mapped poles lie within 15 m of this GNSS midpoint track. The median nearest distance is 5.53 m. That confirms the GNSS export and the pole map are in the same UTM 33N frame, and that this drive passes along the mapped poles. The pole file is still not a vehicle ground-truth trajectory.

## Bag index, payloads not decoded

Chunk payloads were not decompressed.

Reduced bag, 9 topics: `/gps_left_position` (5422), `/gps_right_position` (5422), `/gps_state` (5422), `/utm_position` (5420), `/ouster/imu` (54202), and the four Ouster images `range`, `signal`, `reflec`, `nearir` (5420 each). No point cloud and no `/vehicle_enu_heading`.

Full bag adds `/ouster/points` (5419, the only topic with pure chunks), `/vehicle_enu_heading` (10833), `/vehicle_drive_path`, three camera streams, and Dataspeed MKZ reports. Some chunk-info timestamps in the index are not valid Unix times. The sane chunk window matches the CSV clock. Message counts are the reliable index product.

`gps_state` and `utm_position` would be the right place to confirm the receiver’s UTM zone and fix type. They were not decoded. Per-point LiDAR timing, reflectivity, signal, and near-IR exist as image topics, not as columns in the odometry CSVs.

## Faster-LIO paper, limitations that matter here

Bai et al., RA-L 2022, describe Faster-LIO as FastLIO2’s iterated Kalman filter with incremental voxels (iVox) replacing the ikd-tree. Relevant limits for this experiment, as properties of that method rather than as new claims:

- The estimator is a filter with a local map. The paper’s iVox uses an LRU cache and deletes old voxels. There is no loop closure and no pose-graph, so horizontal error can grow without bound.
- Neighbour search is an approximation inside a fixed voxel neighbourhood, not an exact k-NN. The paper argues this is enough when the IMU initial guess is already good.
- Residuals are the usual point-to-plane kind inherited from FastLIO2. The paper does not analyse geometric degeneracy, roadside repetition, or snow returns.
- Reported accuracy is APE/RPE on public datasets. Several comparison cells are marked failed because of large drift or missing inputs. Snow-covered Nordic roads are outside that evaluation.
- The speed result (on the order of 200 Hz for a spinning lidar on a desktop CPU) is a compute result. It does not bound drift.

Those are the reasons a black-box post-processor is being considered. They are not evidence that a post-processor will succeed on this drive.

## Audit conclusion

The GNSS midpoint is a usable reference track in the same frame as the pole map. It is not survey ground truth. The Faster-LIO export’s two position representations disagree by hundreds of metres and its altitude is not a height. That is a serious coordinate-frame inconsistency. Model training was still run afterwards as a negative experiment and is reported as such in `docs/05_reliability_experiment.md`. It is not an adopted model.
