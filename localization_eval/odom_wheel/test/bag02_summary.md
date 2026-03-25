[INFO] [1774316545.300159819] [rosbag2_storage]: Opened database '/home/dgist/CARLA_ws/bags/test_imu_20260324_103146/test_imu_20260324_103146_0.db3' for READ_ONLY.
Bag  : /home/dgist/CARLA_ws/bags/test_imu_20260324_103146
Mode : offset-correct

=======================================================
  Section 1: AMCL vs Ground Truth  [map frame]
=======================================================
  /carla/hero/odometry samples : 8382
  /amcl_pose samples           : 1057
  map->odom TF samples         : 8394
  Matched samples              : 1056

  Avg position error : 6.3847 m
  Max position error : 33.6824 m
  Avg yaw error      : 0.0132 rad  (0.76 deg)
  Max yaw error      : 0.0692 rad  (3.97 deg)

=======================================================
  Section 2: odom_wheel vs odom_local  [odom frame]
=======================================================
  /odom_local samples  : 16748
  /odom_wheel samples  : 8394
  Matched samples      : 8378

  Avg position error : 0.7176 m
  Max position error : 1.8996 m
  Avg yaw error      : 0.0000 rad  (0.00 deg)
  Max yaw error      : 0.0000 rad  (0.00 deg)

