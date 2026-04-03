# Nav2 Realtime Preflight (CARLA)

## Goal
Prepare everything locally now, then run one short validation session when CARLA server is available.

## 0) Local edits you can do without server

### A. Realtime launch toggle for custom odom TF
File: `src/my_pkg/launch/carla_2d_nav2_realtime.launch.py`

- Added launch arg:
  - `use_relative_odom_tf:=true|false`
- Meaning:
  - `true`: run `odom_tf_from_map_pose` (publishes `odom -> hero`)
  - `false`: disable this node (use external TF chain only)

Use this to A/B test TF conflicts quickly.

### B. Keep map metadata fixed
File: `maps/lidar_launch.yaml`

Check:
- `image` is absolute path
- `resolution`/`origin` are unchanged from map build session

Current expected values:
- `resolution: 0.05`
- `origin: [-80.1, -89.6, 0]`

### C. Nav2 params sanity (no server needed)
File: `src/my_pkg/config/nav2_params.yaml`

Verify these frame settings are consistent:
- `amcl.base_frame_id: hero`
- `amcl.odom_frame_id: odom`
- `amcl.global_frame_id: map`
- `local_costmap.global_frame: odom`
- `local_costmap.robot_base_frame: hero`
- `global_costmap.global_frame: map`
- `global_costmap.robot_base_frame: hero`

Do not change all at once. Keep one baseline config and test one variable at a time later.

## 1) Data to collect once server is connected (single run)

Collect and save outputs in order below.

### A. Runtime topology snapshot
```bash
ros2 topic list | sort
ros2 node list | sort
ros2 action list | sort
```

### B. TF chain availability
```bash
timeout 5s ros2 run tf2_ros tf2_echo map odom
timeout 5s ros2 run tf2_ros tf2_echo odom hero
timeout 5s ros2 run tf2_ros tf2_echo map hero
ros2 topic info /tf -v
```

### C. Rates (performance bottleneck check)
```bash
ros2 topic hz /clock
ros2 topic hz /scan
ros2 topic hz /carla/hero/odometry
```

### D. Nav2 lifecycle and action status
```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator

ros2 topic echo /navigate_to_pose/_action/status --once
ros2 topic echo /navigate_to_pose/_action/feedback --once
```

### E. Planning and control output
```bash
timeout 5s ros2 topic echo /plan --once || echo "no plan"
timeout 5s ros2 topic echo /cmd_vel --once || echo "no cmd_vel"
```

### F. Costmap bounds check
```bash
ros2 topic echo /global_costmap/costmap --once
ros2 topic echo /local_costmap/costmap --once
```

If logs contain `Robot is out of bounds of the costmap` or huge coordinates, capture 30-60s log.

## 2) Recommended test matrix (minimal)

Run only these 2 cases first.

### Case 1 (baseline)
- `use_relative_odom_tf:=true`
- Set initial pose in RViz
- Send one short goal
- Save A~F outputs

### Case 2 (TF conflict check)
- `use_relative_odom_tf:=false`
- Same initial pose/goal procedure
- Save A~F outputs again

Compare:
- Does `map->odom` stay finite and stable?
- Are goal statuses mostly `4` (SUCCEEDED) instead of `6` (ABORTED)?
- Is `/cmd_vel` continuously produced during goal execution?

## 3) Realtime launch command templates

### With custom odom TF ON
```bash
ros2 launch my_pkg carla_2d_nav2_realtime.launch.py use_relative_odom_tf:=true use_rviz:=true
```

### With custom odom TF OFF
```bash
ros2 launch my_pkg carla_2d_nav2_realtime.launch.py use_relative_odom_tf:=false use_rviz:=true
```

## 4) What success should look like

- TF chain is stable: `map -> odom -> hero`
- `Navigation2` panel becomes active after startup and goal
- `/plan` appears soon after goal
- `/cmd_vel` is not stuck at zero during path tracking
- No repeated `out of bounds` warnings

￼
