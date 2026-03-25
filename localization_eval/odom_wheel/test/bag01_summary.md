[INFO] [1774315233.716763604] [rosbag2_storage]: Opened database '/home/dgist/CARLA_ws/bags/test_imu_20260324_100311/test_imu_20260324_100311_0.db3' for READ_ONLY.
Bag  : /home/dgist/CARLA_ws/bags/test_imu_20260324_100311
Mode : offset-correct

=======================================================
  Section 1: AMCL vs Ground Truth  [map frame]
=======================================================
  /carla/hero/odometry samples : 21248
  /amcl_pose samples           : 1096
  map->odom TF samples         : 21271
  Matched samples              : 5293

  Avg position error : 8.4571 m
  Max position error : 32.4340 m
  Avg yaw error      : 0.0159 rad  (0.91 deg)
  Max yaw error      : 0.0658 rad  (3.77 deg)

=======================================================
  Section 2: odom_wheel vs odom_local  [odom frame]
=======================================================
  /odom_local samples  : 42554
  /odom_wheel samples  : 21275
  Matched samples      : 42549

  Avg position error : 1.3399 m
  Max position error : 2.2363 m
  Avg yaw error      : 0.0001 rad  (0.00 deg)
  Max yaw error      : 0.0084 rad  (0.48 deg)

