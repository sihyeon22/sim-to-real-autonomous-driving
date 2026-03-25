# CARLA-ROS2 Nav2 자율주행 검증 프레임워크

**[Project] 소형 모빌리티 자율주행 개발·검증 환경 구축**
퓨처드라이브 (FutureDrive) 산학협력 인턴십 프로젝트

---

## 프로젝트 개요

CARLA 시뮬레이터와 ROS2 Nav2를 연동하여 소형 모빌리티(Ackermann 조향 차량)의 자율주행 알고리즘을 검증하는 환경을 구축한다. 시뮬레이터에서 검증된 알고리즘을 실 플랫폼으로 전환하는 것을 목표로 하며, 이를 위해 실차에서도 사용 가능한 센서 기반 odometry 파이프라인을 핵심으로 개발한다.

---

## 시스템 구성

### 소프트웨어 스택

| 구성 요소 | 버전/사양 |
|---|---|
| OS | Ubuntu 22.04 |
| ROS2 | Humble |
| 시뮬레이터 | CARLA 0.9.x |
| 내비게이션 | Nav2 (Humble) |
| SLAM | slam_toolbox |
| 경로 계획 | SmacPlannerHybrid (DUBIN) |
| 경로 추종 | RegulatedPurePursuitController (RPP) |
| 위치 추정 | AMCL |

### 센서 구성 (CARLA 시뮬레이션)

- **LiDAR**: 차량 상단 장착, pointcloud_to_laserscan으로 2D scan 변환
- **IMU**: 차량 중심, yaw 추정에 활용
- **Speedometer**: 차량 속도 (pseudo 센서)
- **Odometry**: CARLA ground truth (비교 레퍼런스용)

---

## 주요 파이프라인

### Odometry 구조

```
[odom_local]  /carla/hero/odometry (CARLA GT) → odom_tf_from_map_pose → odom→hero TF
[odom_wheel]  /carla/hero/speedometer + /carla/hero/imu → SpeedImuOdomNode → odom→hero TF
```

- **odom_local**: CARLA ground truth 기반, 시뮬레이션 전용 (실차 전환 불가)
- **odom_wheel**: speedometer + IMU dead reckoning 기반, 실차 전환 가능

### Localization 구조

```
odom→hero TF (dead reckoning)
    ↓
AMCL (LiDAR scan matching → map→odom TF 보정)
    ↓
최종 위치: map → odom → hero
```

---

## 런치 파일

| 런치 파일 | Nav2 odom 소스 | 용도 |
|---|---|---|
| `carla_2d_nav2_realtime.launch.py` | odom_local (CARLA GT) | 기본 baseline (이론적 상한선) |
| `carla_2d_nav2_realtime_imu.launch.py` | odom_local | odom_local 주행 + odom_wheel 병렬 발행 (drift 비교) |
| `carla_2d_nav2_wheel_odom.launch.py` | odom_wheel (dead reckoning) | 실차 전환 가능 baseline |
| `carla_2d_slam_realtime.launch.py` | odom_local | 실시간 SLAM 지도 생성 |

---

## 평가 스크립트

```
scripts/
├── compare_localization_bag.py   # AMCL vs CARLA GT 위치 오차 평가 (Section 1)
└── compare_odom_bag.py           # AMCL + odom_wheel vs odom_local 통합 평가 (Section 1+2)
```

### 실행 방법

```bash
# bag 녹화 (주행 중)
ros2 bag record /carla/hero/odometry /amcl_pose -o bags/<bag_name>

# localization 평가
python3 scripts/compare_localization_bag.py bags/<bag_name>/ --mode offset-correct --label <label>

# odom_wheel vs odom_local 비교 평가
python3 scripts/compare_odom_bag.py bags/<bag_name>/ --mode offset-correct
```

---

## 실험 결과 요약 (2026.03.25 기준)

odom_local(CARLA GT)을 레퍼런스로, odom_wheel(dead reckoning)의 주행 성능을 3가지 시나리오에서 비교 평가.

| 시나리오 | AMCL avg error (odom_local) | AMCL avg error (odom_wheel) |
|---|---|---|
| 직선 주행 | 0.732 m | **0.488 m** |
| 곡선 주행 | 1.047 m | 1.344 m |
| 다중 waypoint | 2.047 m | 2.987 m |
| **전체 평균** | **1.275 m** | **1.606 m** |

- 전체 goal 성공률: odom_local 12/12, odom_wheel 12/12 (동일)
- 직선에서는 odom_wheel이 오히려 우세
- 곡선/waypoint에서는 dead reckoning drift 누적으로 성능 저하
- yaw 오차는 전체 평균 기준 거의 동일 (0.0463 vs 0.0462 rad)

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
│   │       └── cmd_vel_to_ackermann.py
│   └── my_pkg_carla_bridge/           # CARLA ROS2 브리지 패키지
├── maps/                              # 저장된 지도 파일
├── scripts/                           # 평가 스크립트
├── localization_eval/                 # 실험 결과
│   ├── odom_local/
│   └── odom_wheel/
└── bags/                              # rosbag 데이터 (gitignore)
```

---

## 향후 개선 방향

- **scan matching 기반 odometry 보정**: rf2o_laser_odometry 등 활용하여 dead reckoning drift 감소
- **복잡 경로 성능 개선**: 곡선/다중 waypoint에서 odom_wheel 성능 격차 축소
- **실차 전환 검증**: 시뮬레이션 파이프라인을 실 플랫폼에 탑재하여 성능 비교
