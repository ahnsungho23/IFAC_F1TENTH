# Localization feature schema

The extractor emits features only. `localization_label` is always `LOC_UNKNOWN`; no CLEAN/BAD
threshold or automatic deletion/move policy exists.

| Column family | Fields | Meaning |
|---|---|---|
| join/time | bag_path, pose_topic, timestamp_ns, localization_label | Read-only source and pose sample key |
| availability | pose_sample_gap_s, timestamp_regression, scan_pose_offset_s, odom_pose_offset_s, frenet_pose_offset_s, imu_pose_offset_s, tf_nearest_offset_s, tf_static_present | Timestamp proximity; nearest-message offset is signed |
| pose continuity | dx_m, dy_m, distance_jump_m, unwrapped_yaw_rad, unwrapped_dyaw_rad, implied_speed_mps, implied_yaw_rate_radps | Consecutive localization samples; no quality threshold |
| Frenet continuity | frenet_s_m, frenet_d_m, ds_m, dd_m, backward_s_jump_m, abs_d_jump_m | Raw consecutive Frenet deltas; closed-track correction is not guessed |
| cross-sensor | odom_speed_mps, localization_minus_odom_speed_mps, imu_yaw_rate_radps, localization_minus_imu_yaw_rate_radps, short_window_s, localization_short_window_displacement_m, odom_short_window_distance_m, short_window_displacement_residual_m | Populated only when actual topics exist |
| MCL | covariance_xx, covariance_yy, covariance_yawyaw, particle_count, particle_x_std_m, particle_y_std_m, particle_yaw_circular_std_rad, particle_multimodality_proxy_m, particle_ess | Split-centroid separation is a threshold-free multimodality proxy; ESS remains blank for PoseArray because it has no weights |
| ground truth | gt_x_m, gt_y_m, gt_yaw_rad, gt_position_error_m, gt_yaw_error_rad | Populated only for exact `/ego_racecar/odom`; Frenet GT error remains unavailable without an explicit GT projection contract |
| unsupported | track_bound_inconsistency, projection_branch_jump_proxy, gt_frenet_s_error_m, gt_frenet_d_error_m | Blank with an explicit availability note; no reference-map branch policy is invented |

All timestamps come from rosbag storage order/time. The tool never writes to the bag directory.
