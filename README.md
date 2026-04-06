# [Project] 소형 모빌리티 자율주행 개발·검증 환경 구축
**퓨처드라이브 (FutureDrive) 산학협력 인턴십 프로젝트**
최종 업데이트: 2026-04-06

---

## 1. 프로젝트 배경 및 목표

소형 모빌리티 분야에서 새로운 센서를 실차에 바로 적용할 경우 물리적 호환성, 성능 검증 부족, 안전사고 위험 등의 문제가 존재한다. 이를 보완하기 위해 **CARLA 시뮬레이터 기반의 사전 검증 체계**를 구축하고, 검증된 알고리즘을 실 플랫폼에 이식하는 프로세스를 확보하는 것이 본 프로젝트의 핵심 목표이다.

### 최종 목표
- CARLA 시뮬레이터 기반 소형 모빌리팉 자율주행 검증 환경 구축
- 시뮬레이터와 실차에서 **동일한 SW 스택**으로 일관된 검증 체계 확보
- 센서(LiDAR, IMU, Camera 등) 적합성 검증 및 인지·경로계획·제어 성능 정량화

---

## 2. 시스템

| 구성 요소 | 사양 |
|----------|------|
| OS | Ubuntu 22.04 |
| ROS2 | Humble |
| 시뮬레이터 | CARLA 0.9.x |
| 내비게이션 | Nav2 (Humble) |
| SLAM | slam_toolbox |
| 경로 계획 | SmacPlannerHybrid (DUBIN) |
| 경로 추종 | RegulatedPurePursuitController (RPP) |
| 위치 추정 | AMCL |
| 차량 모델 | 대형 Charger (초기) → MK-Mini 소형 모빌리티 (현재) |
| LiDAR (시뮬) | CARLA ray_cast, 32ch, z=1.0m, lower_fov=-20° |
| 실차 LiDAR | Hesai OT128 (예정) |

---

## 3. 진행 단계 요약

### Phase 1. CARLA 기본 환경에서 자율주행 파이프라인 구축

CARLA 시뮬레이터에서 기본 제공되는 맵(Town10HD)과 차량 모델(dodge charger police 2020)을 사용하여 ROS2/Nav2 기반 자율주행 파이프라인을 구축

**구축 내용:**
- carla_ros_bridge + carla_spawn_objects 연동
- SLAM 맵 생성 및 저장
- LiDAR(3D PointCloud2) → pointcloud_to_laserscan(2D LaserScan) → AMCL localization
- SmacPlannerHybrid + RPP 기반 Waypoint 자율주행
- GT odometry(odom_local) vs. Custom odometry(odom_wheel: speedometer + IMU dead reckoning) 파이프라인 분리

**odometry 구조:**
```
[odom_local]  /carla/hero/odometry (CARLA GT) → odom_tf_from_map_pose → [/odom→/hero] TF
[odom_wheel]  /carla/hero/speedometer + /carla/hero/imu → SpeedImuOdomNode → [/odom→/hero] TF
```

---

### Phase 2. 실차 유사 환경 적용(R7 주차장 맵 + Mk-Mini 차량 모델)

실차 전환을 목표로 CARLA 내에 MK-Mini 소형 모빌리티 차량 모델을 적용하고, 차량 제원에 맞게 전체 파이프라인 파라미터를 재조정

**MK-Mini 차량 제원:**
- 차체: 길이 0.84m, 너비 0.61m, 높이 0.6m
- wheelbase: 0.6m, 최소 회전 반경: 1.1m
- max_steer_angle: 0.5rad, wheel_radius: 0.12m

**주요 변경 사항:**

| 항목 | 기존 (Charger) | MK-Mini 적용값 |
|------|---------------|---------------|
| LiDAR z | 2.4m | 1.0m |
| lower_fov | -45° | -20° (지면 반사 감소) |
| min_height | -2.35 | -0.89~-0.95 |
| max_height | -1.8 | -0.05 |
| range_max | 15m | 8m |
| footprint | [2.6, 1.05] | [0.42, 0.305] |
| minimum_turning_radius | 3.0m | 1.1m |
| wheelbase (cmd_vel_to_ackermann) | 3.0m | 0.6m |
| acceleration | 0.8 | 0.0 (speed PID 모드) |

**런치 파일 구성:**

| 파일 | 용도 | odom 소스 |
|------|------|-----------|
| `baseline_mkmini.launch.py` | MK-Mini 자율주행 | Custom odom (실차 전환 대상) |
| `mkmini_GT.launch.py` | MK-Mini 자율주행 | GT odom (upper bound 레퍼런스) |
| `mkmini_2d_slam.launch.py` | MK-Mini SLAM 맵 생성 | GT odom |

---

### Phase3. 변수 비교 실험 - Localization 오차 분석

동일 환경에서 변수를 달리해 AMCL localization 오차를 정량 비교

#### 실험 1. 제어 방식 비교 (Ackermann vs. Twist)

- Ackermann: /cmd_vel → cmd_vel_to_ackermann → carla_ackermann_control(PID)
- Twist: /cmd_vel → carla_twist_to_control(단순 throttle 매핑)
- bags: `bags/ackermann/run01~03`, `bags/twist/run01~03`
- 분석: `scripts/plot_control_compare.py`

#### 실험 2. IMU Noise 수준 비교

- low / mid / high noise 조건에서 localization 오차 비교
- bags: `bags/imu/test/low_noise`, `mid_noise`, `high_noise`
- 분석: `scripts/plot_imu_noise_compare.py --labels LOW MID HIGH`

#### 실험 3. AMCL Particle Count 비교

- 500/2000, 1000/5000, 2000/10000 비교
- bags: `bags/imu/particle_test/run01~03`
- 분석: `scripts/plot_imu_noise_compare.py --labels "500/2000" "1000/5000" "2000/10000"`

#### 실험 결과 요약 (dodge charger police 모델 기준)

---

## 파이프라인 구조

---

## 향후 계획

### 실차 드라이버 개발 (표준 입출력 계층 설계)

현재 carla_ros_bridge에 직접 의존하는 구조를 개선하여, 시뮬레이터/실차 공통으로 사용 가능한 **표준 ROS2 입출력 계층**을 설계한다.

```
[시뮬]                          [실차]
carla_ros_bridge                 각 센서 vendor driver
      ↓                                 ↓
 sim_adapter               sensor_adapter_lidar / imu / camera / vehicle
      ↓                                 ↓
                    /sensor/*
                        ↓
               Nav2 (표준 토픽만 구독)
```

**개발 컴포넌트:**
- `sim_adapter`: CARLA 토픽 → 표준 토픽 변환
- `sensor_adapter_lidar`: 실차 LiDAR 출력 → 표준 PointCloud
- `sensor_adapter_imu`: 실차 IMU 출력 → 표준 IMU
- `vehicle_adapter`: 제어 데이터 → 차량 상태/제어 msg

**예상 리스크:**
- timestamp 동기화
- TF/frame 체계 설계
- 센서 좌표계 통일
- 제어 메시지 정의
- 센서별 QoS/주기 차이
- 실차 네트워크 지연/패킷 손실 처리
- calibration 연동

**선행 확인 사항:**
- MK-Mini 실차 센서 목록 및 드라이버 출력 메시지 타입
- 제어 관련 메시지

---

## 디렉토리 구조

```
CARLA_ws/
├── src/
│   ├── my_pkg/                        # 메인 패키지
│   │   ├── launch/                    # 런치 파일
│   │   ├── config/                    # Nav2 파라미터, BT XML
│   │   └── my_pkg/                    # 노드 소스코드
│   │       ├── odometry.py            # SpeedImuOdomNode (dead reckoning)
│   │       ├── odom_tf_from_map_pose.py
│   │       ├── cmd_vel_to_ackermann.py
│   │       └── waypoint_sender.py
│   └── my_pkg_carla_bridge/           # CARLA ROS2 브릿지
│       ├── carla_ros_bridge/
│       ├── carla_spawn_objects/
│       │   └── config/object_mkmini3_parking.json
│       └── carla_ackermann_control/
├── maps/parking/                      # MK-Mini 주차장 맵
├── scripts/                           # 분석 스크립트
├── bags/                              # rosbag 데이터
│   ├── ackermann/
│   ├── twist/
│   └── imu/
└── sim_log/                           # 런치 로그
```