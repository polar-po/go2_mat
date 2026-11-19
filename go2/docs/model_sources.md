# Go2 모델 출처와 파라미터 확인

## 고정한 원본

- **기준 모델**은 Unitree `unitree_mujoco`의 커밋 `1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d`입니다. 고정 SHA에서 다운로드한 XML 원본을 [reference/go2.xml](../reference/go2.xml)에 보관했습니다. SHA-256은 `2014a3d76e30f17ab9447d8a67bd015291f74fa4d71ae30d005f1a32bd693d4b`입니다. 원본 XML은 별도 메시 파일을 참조합니다. Phase 0의 파라미터 검증에는 XML만 사용했습니다. [Unitree Go2 MJCF](https://github.com/unitreerobotics/unitree_mujoco/blob/1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d/unitree_robots/go2/go2.xml) · [Unitree BSD 3-Clause 라이선스](../reference/LICENSE-unitree_mujoco)
- **교차검증 모델**은 Unitree `unitree_ros`의 커밋 `157ff950351e5f535e03c5d85a16b7d594b3dbcc`입니다. 해당 SHA에서 URDF를 다운로드해 XML 구조를 읽고 네 다리의 관절값을 대조했습니다. 프로젝트에는 기준 MJCF 원본을 보관합니다. [Unitree Go2 URDF](https://github.com/unitreerobotics/unitree_ros/blob/157ff950351e5f535e03c5d85a16b7d594b3dbcc/robots/go2_description/urdf/go2_description.urdf)

## 기준 모델에서 확인한 값

| 항목 | 값과 해석 |
| --- | --- |
| 고관절 원점 B [m] | FL `[+0.1934, +0.0465, 0]`, FR `[+0.1934, -0.0465, 0]`, RL `[-0.1934, +0.0465, 0]`, RR `[-0.1934, -0.0465, 0]` |
| 링크 오프셋 [m] | Hip→Thigh는 부모 링크의 Y축으로 왼쪽 `+0.0955`, 오른쪽 `-0.0955`. Thigh→Calf와 Calf→발 프레임은 각각 부모 링크 좌표로 `[0, 0, -0.213]`. 관절각 0에서 링크 축은 부모 축과 정렬됩니다. |
| 관절 회전축 | 관절의 로컬 축을 기준으로 hip +X, thigh +Y, calf +Y |
| 발 충돌 구 [m] | Calf 프레임 기준 중심 `[-0.002, 0, -0.213]`, 반지름 `0.022`. 접촉 위치는 지면의 법선과 발 구의 기하로 별도 계산합니다. |
| 각도 범위 [rad] | Hip `[-1.0472, 1.0472]`, 앞 thigh `[-1.5708, 3.4907]`, 뒤 thigh `[-0.5236, 4.5379]`, calf `[-2.7227, -0.83776]` |
| 모델 모터 제어 범위 [N·m] | Hip 및 thigh ±23.7, calf ±45.43. MJCF의 단위 전달비 모터 제어 범위이며, 실물 모터의 연속 정격은 별도 자료가 필요합니다. |
| 질량 [kg] | 몸체 6.921. 다리당 hip 0.678, thigh 1.152, calf 0.241352. XML의 명시적 관성 질량 합은 15.206408. |
| 홈 관절각 [rad] | MJCF의 `home` 키프레임에서 네 다리 모두 hip 0, thigh 0.9, calf −1.8입니다. Phase 1의 평지 자세는 이 각도를 사용하고 몸체 높이는 발 구의 FK로 계산합니다. |

위 값은 각 다리의 링크 계층, 관절에 지정된 기본 클래스와 상속된 축·범위, 모터 클래스, 발 충돌 구 클래스, 관성 질량을 읽어서 확인했습니다. MJCF 액추에이터의 나열 순서는 FR, FL, RR, RL입니다. MATLAB 구조체의 순서는 FL, FR, RL, RR이며 나중에 데이터를 연결할 때 관절 이름으로 매핑할 수 있습니다.

URDF는 12개 관절의 원점, 회전축과 각도 범위 및 네 발 고정 관절의 원점이 MJCF와 일치합니다. URDF의 발 구 중심은 발 프레임에서 `[-0.002, 0, 0]` m이며 반지름은 0.022 m입니다. 발 프레임 오프셋을 더하면 MJCF와 같은 중심이 됩니다. URDF의 기본 링크 질량과 다리별 hip·thigh 질량은 MJCF와 같고, URDF의 calf와 foot 항목은 각각 0.154 kg, 0.04 kg입니다. MATLAB의 질량 합산에는 MJCF의 관성 항목을 사용했습니다.

## 단순화와 예시

- **단순화 가정**으로 질량 15.206408 kg을 B 원점 `[0; 0; 0]`에 집중시킵니다. 개별 링크 중력이나 자세에 따른 전체 질량중심 이동은 이 근사에 포함되지 않습니다.
- **시각화 근사**로 몸통 크기 `[0.45; 0.18; 0.12]` m를 사용합니다. X, Y, Z 방향 박스 길이입니다.
- **중력 설정**은 세계 좌표계 기준 `[0; 0; -9.81]` m/s²입니다.
- **실습 예시**로 초기 마찰계수 `mu = 0.5`를 선택했습니다. 지면 실측 데이터가 생기면 시나리오에 맞춰 입력할 수 있습니다.
- **기준 모델의 배치 예**로 XML의 `base_link` 높이는 0.445 m이고 `home` 키프레임의 몸체 높이는 0.27 m입니다. MATLAB 데모의 기준 지면은 `z_W = 0`이며, 데모의 몸체 높이는 발 구 중심의 FK와 구 반지름으로 구합니다.
