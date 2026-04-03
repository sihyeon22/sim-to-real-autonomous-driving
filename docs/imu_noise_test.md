# IMU 노이즈 테스트 결과

## 테스트 목적
실차 전환 시 IMU 노이즈/drift가 발생했을 때 AMCL이 어느 수준까지 보정 가능한지 검증.

## 환경
- Launch: `carla_2d_nav2_wheel_odom.launch.py`
- Nav2 params: `nav2_params_parking.yaml`
- Odom: `/odom_wheel` (speedometer + IMU angular_velocity.z 적분)
- 주행: `waypoint_sender` (WP1 → WP2)
- 평가: `scripts/compare_localization_bag.py` (offset-correct mode)
- 평가 기준: AMCL pose vs CARLA ground truth (`/carla/hero/odometry`)

현재 테스트는 `odometry.py`를 `angular_velocity.z` 적분 방식으로 수정 후 진행.
gyro 노이즈/bias가 odom yaw drift로 직접 반영됨.

## 노이즈 파라미터 설명
| 파라미터 | 의미 |
|----------|------|
| `noise_gyro_stddev_x/y/z` | 매 순간 gyroscope에 더해지는 랜덤 노이즈(yaw 추정 오차) — 값 클수록 yaw 불규칙 떨림 |
| `noise_gyro_bias_x/y/z` | 항상 같은 방향으로 쌓이는 yaw rate 오차(yaw drift) — 값 클수록 yaw 한쪽으로 지속 drift |

## 결과
| 단계 | noise_gyro_stddev | noise_gyro_bias | Avg pos error | Max pos error | Avg yaw error | Max yaw error | bag |
|------|-------------------|-----------------|---------------|---------------|---------------|---------------|-----|
| baseline | 0.0 | 0.0 | 0.506 m | 1.198 m | 0.0167 rad (0.95°) | 0.1173 rad (6.72°) | bags/imu/imu_noise_baseline |
| 약한 노이즈 | 0.01 | 0.001 | 0.578 m | 1.389 m | 0.0180 rad (1.03°) | 0.1104 rad (6.32°) | bags/imu/low_noise |
| 중간 노이즈 | 0.05 | 0.005 | 1.990 m | 4.539 m | 0.0628 rad (3.60°) | 0.1669 rad (9.56°) | bags/imu/mid_noise |
| 강한 노이즈 | 0.1 | 0.01 | 4.492 m | 8.642 m | 0.1308 rad (7.49°) | 0.2477 rad (14.19°) | bags/imu/high_noise |

## 분석
- **baseline**: 노이즈 없어도 적분 누적 오차로 Avg 0.5m 수준 오차 발생. AMCL이 어느 정도 보정하고 있음.
- **약한 노이즈**: baseline 대비 소폭 증가(0.58m). AMCL 보정 범위 내. 실차 수준의 저품질 IMU와 유사.
- **중간 노이즈**: Avg 2.0m로 baseline 대비 4배 증가. Max 4.5m는 AMCL 보정 한계 근처. local costmap 틀어짐 육안 확인.
- **강한 노이즈**: Avg 4.5m, Max 8.6m. AMCL 보정 한계를 벗어나 주행 품질 크게 저하.

## 결론
- AMCL pose 오차가 노이즈 증가에 따라 4배 이상 커지는 것은, AMCL이 odom drift를 완전히 보정하지 못함을 의미.
- 모든 노이즈 수준에서 waypoint 도달(주행 성공) 자체는 가능했음.
- 주행 품질 차이는 정량 지표보다 육안 관찰로 확인: 노이즈가 낮을수록 local costmap 정합이 안정적이고 회전 동작이 매끄러움.
- 중간(0.05) 이상에서는 local costmap 틀어짐이 육안으로 뚜렷이 확인됨.
- 실차 전환 시 IMU 품질에 따라 localization 안정성이 달라질 수 있으며, 추가 검증 필요.

---

## Particle 수 증가 효과 테스트

### 조건
- 노이즈: 중간 노이즈 (noise_gyro_stddev=0.05, noise_gyro_bias=0.005)
- 비교: `min_particles`/`max_particles` 500/2000 (기본값) vs 1000/5000 (증가)
- bag: `bags/imu/baseline/mid_noise` vs `bags/imu/baseline/mid_noise_particle_inc`

### 결과
| 설정 | Avg pos error | Max pos error | Avg yaw error | Max yaw error |
|------|---------------|---------------|---------------|---------------|
| 500/2000 (기본값) | 11.456 m | 20.283 m | 0.3763 rad (21.56°) | 0.4688 rad (26.86°) |
| 1000/5000 (증가) | 3.728 m | 7.862 m | 0.1305 rad (7.47°) | 0.4632 rad (26.54°) |

### 분석
- Avg pos error 67% 감소 (11.5m → 3.7m), Max pos error 61% 감소 (20.3m → 7.9m)
- Avg yaw error 65% 감소 (21.6° → 7.5°)
- Max yaw error는 두 설정 모두 유사 (26.9° vs 26.5°) — 순간적인 최대 오차는 particle 수로 해결 안 됨
- 중간 노이즈 조건에서 particle 수 증가가 localization 품질에 큰 효과 있음
- 단, 연산 부하 증가 수반 — 실차 적용 시 CPU 여유 확인 필요
