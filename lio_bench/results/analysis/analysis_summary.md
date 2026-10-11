# Step 3 - weakness analysis

54 windows. Mounting yaw removed (deg): {"fast_lio": 1.11, "faster_lio": 1.13, "dlio": 1.07, "genz_lio": 1.08}

## Errors per method (10 s windows)

| method | scale err median | scale |p90| | lateral median | yaw drift median deg | step err median m |
|---|---:|---:|---:|---:|---:|
| fast_lio | -0.13% | 0.76% | -0.13% | 0.44 | 0.058 |
| faster_lio | -0.17% | 0.87% | -0.10% | 0.48 | 0.059 |
| dlio | 0.13% | 0.51% | -0.03% | 0.26 | 0.156 |
| genz_lio | -0.11% | 0.81% | -0.08% | 0.47 | 0.057 |

## What the errors go with (Spearman rho, top 6)

- **fast_lio__trans_err**: genz_lio__trans_err (+0.95), can_speed_mps (+0.40), genz_scale_indicator (+0.37), genz_median_range (+0.37), scan_imu_acc_norm_std (+0.37), gt_speed (+0.34)
- **fast_lio__scale_err**: scan_z_std (-0.46), genz_voxelized_points (-0.45), genz_point_matches (-0.44), genz_lio__scale_err (-0.44), genz_processing_time_ms (-0.35), scan_range_p50 (-0.35)
- **fast_lio__yaw_err_deg**: genz_lio__yaw_err_deg (+0.52), genz_lio__lat_err (+0.43), genz_plane_matches (+0.29), genz_cov_rot_trace (-0.26), scan_imu_yaw_rate (+0.25), genz_lio__trans_err (-0.24)
- **faster_lio__trans_err**: genz_lio__trans_err (+0.97), can_speed_mps (+0.38), scan_imu_acc_norm_std (+0.35), genz_scale_indicator (+0.34), genz_median_range (+0.34), gt_speed (+0.33)
- **faster_lio__scale_err**: genz_lio__scale_err (-0.59), scan_z_std (-0.52), genz_voxelized_points (-0.45), genz_processing_time_ms (-0.44), genz_median_range (-0.43), genz_scale_indicator (-0.42)
- **faster_lio__yaw_err_deg**: genz_lio__yaw_err_deg (+0.62), genz_lio__lat_err (+0.44), genz_point_matches (-0.30), genz_plane_matches (+0.29), scan_n_points (+0.27), scan_return_ratio (+0.27)
- **dlio__trans_err**: can_speed_mps (+0.70), gt_speed (+0.68), genz_scale_indicator (+0.65), genz_median_range (+0.64), scan_frac_far30 (+0.54), genz_cov_pos_trace (+0.54)
- **dlio__scale_err**: genz_point_matches (-0.38), genz_voxelized_points (-0.32), scan_planar_frac (+0.31), genz_lio__lat_err (-0.30), genz_processing_time_ms (-0.28), genz_leaf_size (-0.26)
- **dlio__yaw_err_deg**: can_steering_wheel_angle_rad (+0.38), genz_lio__yaw_err_deg (+0.35), scan_imu_yaw_rate (+0.32), genz_point_matches (-0.32), gt_turn_deg (+0.30), genz_lio__trans_err (-0.30)
- **genz_lio__trans_err**: genz_lio__trans_err (+1.00), scan_imu_acc_norm_std (+0.40), genz_scale_indicator (+0.39), genz_median_range (+0.39), can_speed_mps (+0.34), scan_range_p90 (+0.29)
- **genz_lio__scale_err**: genz_lio__scale_err (-0.49), genz_processing_time_ms (-0.45), scan_planar_frac (+0.37), genz_point_matches (-0.36), scan_imu_acc_norm_std (-0.31), genz_voxelized_points (-0.30)
- **genz_lio__yaw_err_deg**: genz_lio__yaw_err_deg (+0.69), scan_range_p90 (-0.37), genz_lio__lat_err (+0.36), genz_scale_indicator (-0.34), genz_median_range (-0.34), scan_n_points (+0.34)

## Median step error by regime (tertiles)

- **gt_speed** edges {'low': {'min': 0.417, 'max': 20.849}, 'mid': {'min': 20.867, 'max': 20.961}, 'high': {'min': 20.965, 'max': 21.08}}: {'fast_lio': {'low': 0.0528, 'mid': 0.0597, 'high': 0.0599}, 'faster_lio': {'low': 0.053, 'mid': 0.0578, 'high': 0.0594}, 'dlio': {'low': 0.1229, 'mid': 0.1599, 'high': 0.1608}, 'genz_lio': {'low': 0.0542, 'mid': 0.0574, 'high': 0.0586}}
- **gt_turn_deg** edges {'low': {'min': 0.034, 'max': 6.747}, 'mid': {'min': 7.006, 'max': 13.989}, 'high': {'min': 14.144, 'max': 101.685}}: {'fast_lio': {'low': 0.0599, 'mid': 0.0598, 'high': 0.0518}, 'faster_lio': {'low': 0.0587, 'mid': 0.0598, 'high': 0.0517}, 'dlio': {'low': 0.1467, 'mid': 0.1571, 'high': 0.1592}, 'genz_lio': {'low': 0.0577, 'mid': 0.0596, 'high': 0.0523}}
- **scan_trans_cond** edges {'low': {'min': 0.003, 'max': 0.022}, 'mid': {'min': 0.023, 'max': 0.041}, 'high': {'min': 0.041, 'max': 0.205}}: {'fast_lio': {'low': 0.0549, 'mid': 0.0597, 'high': 0.0619}, 'faster_lio': {'low': 0.0543, 'mid': 0.0592, 'high': 0.0614}, 'dlio': {'low': 0.167, 'mid': 0.1569, 'high': 0.138}, 'genz_lio': {'low': 0.0553, 'mid': 0.058, 'high': 0.0609}}
- **scan_return_ratio** edges {'low': {'min': 0.057, 'max': 0.31}, 'mid': {'min': 0.324, 'max': 0.468}, 'high': {'min': 0.5, 'max': 0.695}}: {'fast_lio': {'low': 0.0597, 'mid': 0.0587, 'high': 0.0528}, 'faster_lio': {'low': 0.0596, 'mid': 0.0579, 'high': 0.053}, 'dlio': {'low': 0.164, 'mid': 0.1544, 'high': 0.1315}, 'genz_lio': {'low': 0.0596, 'mid': 0.0571, 'high': 0.0542}}
- **scan_range_p50** edges {'low': {'min': 12.907, 'max': 15.732}, 'mid': {'min': 15.773, 'max': 22.552}, 'high': {'min': 22.941, 'max': 48.477}}: {'fast_lio': {'low': 0.0598, 'mid': 0.0537, 'high': 0.0597}, 'faster_lio': {'low': 0.0581, 'mid': 0.053, 'high': 0.0593}, 'dlio': {'low': 0.1453, 'mid': 0.1399, 'high': 0.1683}, 'genz_lio': {'low': 0.0569, 'mid': 0.0542, 'high': 0.0592}}
- **scan_planar_frac** edges {'low': {'min': 0.129, 'max': 0.357}, 'mid': {'min': 0.357, 'max': 0.526}, 'high': {'min': 0.539, 'max': 0.875}}: {'fast_lio': {'low': 0.0619, 'mid': 0.057, 'high': 0.0569}, 'faster_lio': {'low': 0.0614, 'mid': 0.0566, 'high': 0.0563}, 'dlio': {'low': 0.133, 'mid': 0.1574, 'high': 0.1592}, 'genz_lio': {'low': 0.0609, 'mid': 0.057, 'high': 0.056}}

## Best method share of windows

{'faster_lio': 0.453, 'genz_lio': 0.358, 'fast_lio': 0.189}

