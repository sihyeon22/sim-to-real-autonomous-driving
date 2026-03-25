#!/usr/bin/env bash

# 1. ROS2 및 워크스페이스 환경 설정
source /opt/ros/humble/setup.bash
source ~/carla-ros-bridge-ws/install/setup.bash

echo "[Simulation Start]"

# 2. 수동 제어권 강제 획득
ros2 topic pub -1 /carla/hero/vehicle_control_manual_override std_msgs/msg/Bool "{data: true}" &
OVERRIDE_PID=$!
sleep 1

# 3. 주행 시뮬레이션 명령어 그대로 순차 실행
echo "Step 1: throttle: 0.35, steer: 0.0"
timeout 20s ros2 topic pub -r 20 /carla/hero/vehicle_control_cmd_manual carla_msgs/msg/CarlaEgoVehicleControl \
  "{throttle: 0.35, steer: 0.0, brake: 0.0, hand_brake: false, reverse: false, manual_gear_shift: false, gear: 1}"

echo "Step 2: throttle: 0.30, steer: 0.05"
timeout 15s ros2 topic pub -r 20 /carla/hero/vehicle_control_cmd_manual carla_msgs/msg/CarlaEgoVehicleControl \
  "{throttle: 0.30, steer: 0.05, brake: 0.0, hand_brake: false, reverse: false, manual_gear_shift: false, gear: 1}"

echo "Step 3: throttle: 0.30, steer: 0.10"
timeout 25s ros2 topic pub -r 20 /carla/hero/vehicle_control_cmd_manual carla_msgs/msg/CarlaEgoVehicleControl \
  "{throttle: 0.30, steer: 0.10, brake: 0.0, hand_brake: false, reverse: false, manual_gear_shift: false, gear: 1}"

echo "Step 4: throttle: 0.35, steer: 0.0"ㅇrla_msgs/msg/CarlaEgoVehicleControl \
  "{throttle: 0.35, steer: 0.0, brake: 0.0, hand_brake: false, reverse: false, manual_gear_shift: false, gear: 1}"

echo "Step 5: throttle: 0.30, steer: 0.12"
timeout 13s ros2 topic pub -r 20 /carla/hero/vehicle_control_cmd_manual carla_msgs/msg/CarlaEgoVehicleControl \
  "{throttle: 0.30, steer: 0.11, brake: 0.0, hand_brake: false, reverse: false, manual_gear_shift: false, gear: 1}"

echo "Step 6: throttle: 0.35, steer: 0.0"
timeout 8s ros2 topic pub -r 20 /carla/hero/vehicle_control_cmd_manual carla_msgs/msg/CarlaEgoVehicleControl \
  "{throttle: 0.35, steer: 0.0, brake: 0.0, hand_brake: false, reverse: false, manual_gear_shift: false, gear: 1}"

echo "Step 7: brake: 1.0"
ros2 topic pub -1 /carla/hero/vehicle_control_cmd_manual carla_msgs/msg/CarlaEgoVehicleControl \
  "{throttle: 0.0, steer: 0.0, brake: 1.0, hand_brake: false, reverse: false, manual_gear_shift: false, gear: 1}"
  
# 오버라이드 프로세스 종료
kill $OVERRIDE_PID
echo "[Simulation End]"