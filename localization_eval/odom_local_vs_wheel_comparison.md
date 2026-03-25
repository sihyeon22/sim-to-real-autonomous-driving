# `odom_local` vs `odom_wheel` 주행 성능 및 품질 비교

기준 데이터:

- 주행 로그: `/home/dgist/CARLA_ws/sim_log/odom_local`, `/home/dgist/CARLA_ws/sim_log/odom_wheel`
- 로컬라이제이션 평가: `/home/dgist/CARLA_ws/localization_eval/odom_local`, `/home/dgist/CARLA_ws/localization_eval/odom_wheel`

비교 기준:

- 성공 여부: `sim_log`에서 `Begin navigating`, `Goal succeeded`, `Goal canceled`, planner abort 로그 기준
- 주행 시간: `Begin navigating -> Goal succeeded`
- `waypoint`: 성공한 각 waypoint 구간 시간의 합
- 품질 지표: `localization_eval`의 AMCL vs Ground Truth
  - Avg/Max position error
  - Avg/Max yaw error

## 주행별 비교

| 시나리오/주행 | `odom_local` | `odom_wheel` | 비교 요약 |
|---|---|---|---|
| straight/run01 | 성공, 183.8s, 평균 위치오차 0.494m, 평균 yaw 0.0256rad | 성공, 250.6s, 평균 위치오차 0.273m, 평균 yaw 0.0284rad | `wheel`이 위치정확도 우세, `local`이 더 빠름 |
| straight/run02 | 성공, 291.6s, 0.586m, 0.0247rad | 성공, 277.8s, 0.572m, 0.0311rad | 위치오차는 비슷, `wheel`이 약간 빠름, `local` yaw 우세 |
| straight/run03 | 성공, 392.7s, 1.115m, 0.0266rad | 성공, 154.5s, 0.617m, 0.0305rad | `wheel`이 크게 우세 |
| curve/run01 | 성공, 218.4s, 0.987m, 0.0568rad | 성공, 368.4s, 1.317m, 0.0361rad | 위치는 `local`, yaw는 `wheel`, 시간은 `local` 우세 |
| curve/run02 | 성공, 555.5s, 1.220m, 0.0544rad | 성공, 477.5s, 1.271m, 0.0463rad | 위치는 비슷하게 `local` 근소 우세, 시간/yaw는 `wheel` 우세 |
| curve/run03 | 성공, 403.0s, 0.933m, 0.0430rad | 성공, 418.1s, 1.443m, 0.0555rad | `local` 우세 |
| waypoint/run01 | 부분실패, 751.7s, 2.234m, 0.0637rad | 성공, 716.2s, 3.109m, 0.0698rad | `local` 정확도는 더 좋지만 planner abort 다수, 완주 안정성은 `wheel` 우세 |
| waypoint/run02 | 성공, 690.2s, 2.282m, 0.0709rad | 성공, 764.6s, 3.394m, 0.0639rad | 위치는 `local`, yaw는 `wheel`, 시간은 `local` 우세 |
| waypoint/run03 | 성공, 558.2s, 1.626m, 0.0506rad | 부분실패, 941.7s, 2.456m, 0.0541rad | `local` 우세 |

## 시나리오별 평균 비교

| 시나리오 | `odom_local` 평균 | `odom_wheel` 평균 | 해석 |
|---|---|---|---|
| straight | 위치오차 0.732m, yaw 0.0256rad, 289.4s | 위치오차 0.488m, yaw 0.0300rad, 227.6s | 직선은 `wheel`이 전반적으로 더 좋음 |
| curve | 위치오차 1.047m, yaw 0.0514rad, 392.3s | 위치오차 1.344m, yaw 0.0460rad, 421.3s | 곡선은 `local`이 위치/시간 우세, `wheel`은 yaw 우세 |
| waypoint | 위치오차 2.047m, yaw 0.0617rad, 666.7s | 위치오차 2.987m, yaw 0.0626rad, 807.5s | waypoint는 `local`이 정확도와 시간에서 우세, 둘 다 안정성 이슈 존재 |

## 전체 평균

| 항목 | `odom_local` | `odom_wheel` | 우세 |
|---|---|---|---|
| Avg position error | 1.275m | 1.606m | `odom_local` |
| Avg max position error | 4.051m | 4.628m | `odom_local` |
| Avg yaw error | 0.0463rad | 0.0462rad | 거의 동일 |
| Avg max yaw error | 0.2586rad | 0.2272rad | `odom_wheel` |
| Avg duration | 449.5s | 485.5s | `odom_local` |

## 안정성 관찰

- `odom_local`
  - `waypoint/run01`에서 planner abort가 반복적으로 발생
  - 로그상 `Goal canceled`, `Failed to get result`, `Starting point in lethal space`가 확인됨
  - 대신 curve/waypoint 전체 평균 정확도는 더 좋음
- `odom_wheel`
  - 직선에서는 가장 안정적이고 빠른 편
  - `waypoint/run03`에서 초기 경로 계산 실패 흔적이 있으나 최종 goal은 성공
  - 곡선/waypoint에서는 위치오차가 `odom_local`보다 전반적으로 큼

## 최종 결론

- 직선 주행 중심이면 `odom_wheel`이 더 유리함
  - 더 낮은 평균 위치오차
  - 더 짧은 평균 주행 시간
- 곡선과 waypoint를 포함한 복합 주행이면 `odom_local`이 더 유리함
  - 더 낮은 평균 위치오차
  - 더 짧은 평균 주행 시간
  - 특히 waypoint에서 정확도 차이가 큼
- yaw 관점에서는 큰 차이가 없지만, 최대 yaw 오차는 `odom_wheel`이 약간 더 안정적임
- 실제 운영 관점에서는
  - 단순 직선 경로: `odom_wheel`
  - 복합 경로 및 전체 품질 우선: `odom_local`

