# Unitree Go2 기구학과 접촉 정역학

MATLAB R2025a에서 Go2의 관절각과 발 위치 사이의 관계를 직접 계산하고, 네 발로 서 있는 자세가 지면반력으로 힘과 모멘트 평형을 이룰 수 있는지 살펴보는 학습 프로젝트입니다. Phase 1에서는 공식 모델의 파라미터로 네 다리의 순기구학(FK)을 계산하고 자세를 3D로 비교합니다.

## MATLAB에서 실행

MATLAB의 현재 폴더를 이 프로젝트의 루트인 `go2`로 맞춘 다음 명령 창에 입력합니다.

```matlab
main
run_tests
p = go2.model.parameters();
p.geometry.hipOriginB
```

`main`은 모델 요약과 발 위치를 출력하고, 기본 자세와 앞왼쪽(FL)의 세 관절각을 바꾼 자세를 나란히 표시합니다. `main(false)`는 Phase 0의 모델 요약만 출력합니다. `p`는 관련 값을 이름별로 묶는 MATLAB 구조체입니다. `p.geometry.hipOriginB`는 네 고관절의 몸체 기준 좌표를 열 하나씩 보여줍니다. `+go2`와 `+model`은 `go2.model`이라는 함수 이름을 만드는 패키지 폴더입니다. MATLAB 검색 경로에는 프로젝트 루트를 추가합니다.

터미널에서도 그래픽 창 없이 실행할 수 있습니다.

```powershell
matlab -wait -batch "run_tests"
```

## 좌표와 단위

- 세계 좌표계 W와 몸체 좌표계 B는 오른손 좌표계입니다. 기준 축은 +X 전방, +Y 왼쪽, +Z 위쪽입니다. 몸체에 붙은 축은 몸체가 회전하면 함께 회전합니다.
- `R_WB`는 몸체 기준 벡터를 세계 기준으로 회전시킵니다. 위치는 `p_W = p_WB + R_WB * p_B`로 변환합니다.
- 길이 m, 질량 kg, 시간 s, 각도 rad, 힘 N, 토크 N·m를 씁니다. 세계 좌표계의 중력 가속도는 `[0; 0; -9.81]` m/s²로 둡니다.
- 다리 순서는 FL(앞왼쪽), FR(앞오른쪽), RL(뒤왼쪽), RR(뒤오른쪽)입니다. 관절각 데이터는 4×3이며 열 순서는 hip, thigh, calf입니다.

## 순기구학 계산

```matlab
p = go2.model.parameters();
state = go2.model.initialState(p);
fl = go2.kin.forwardLeg(p, 1, state.q(1,:));
robot = go2.kin.forwardRobot(p, state);
robot.footPosW
```

- `state.base.pos`는 세계 좌표계에서 몸체 원점의 위치(3×1), `state.base.R`은 B에서 W로 변환하는 회전행렬(3×3)입니다. `state.q`는 rad 단위의 관절각(4×3)입니다.
- 한 다리의 `jointPosB`는 3×3이며 hip, thigh, calf의 위치가 열마다 들어갑니다. 두 발 위치 `footPosB`와 `footSphereCenterB`는 각각 3×1입니다.
- 네 다리 결과의 `jointPosB`와 `jointPosW`는 3×3×4입니다. 예를 들어 `robot.jointPosW(:,3,2)`는 세계 좌표계에서 FR의 calf 관절 위치입니다. 발 프레임 원점 `footPosB/W`와 발 충돌 구 중심 `footSphereCenterB/W`는 각각 3×4입니다.
- 부모 링크 기준 오프셋을 더해 다음 관절 원점을 구하고, 그 위치에서 로컬 관절축으로 회전합니다. 몸체 기준 누적 회전은 hip에서 `Rx(qHip)`, thigh에서 `Rx(qHip)*Ry(qThigh)`, calf에서 `Rx(qHip)*Ry(qThigh)*Ry(qCalf)`입니다. 마지막에 `p_W = p_WB + R_WB*p_B`를 적용합니다.
- 홈 자세라면 `initialState`가 평지 `z_W = 0`에서 네 발 구가 닿도록 몸체 높이를 계산합니다. 발 프레임 원점, 구 중심, 지면에 닿는 지점은 서로 구분합니다. 데모의 관절각 변경은 고정된 몸체에서의 기하 변화입니다.
- FK는 영 관절각처럼 모델의 관절 한계 밖에 있는 입력도 계산할 수 있습니다. 초기 자세가 관절 한계 안에 있는지는 테스트에서 별도로 확인합니다.

## Phase 1 검증 결과

MATLAB R2025a Update 1에서 Phase 0 테스트 5개와 Phase 1 테스트 11개가 모두 통과했습니다. 3D 데모도 실행해 두 그림의 렌더링과 축 비율을 확인했습니다. 몸체 높이는 0.288373 m이고 홈 자세에서 네 발 구의 평지 상대 간격은 반올림 오차 범위였습니다(약 2.08e-17 m). 이 결과는 평지와 발 충돌 구의 기하 관계를 검증합니다.

## Phase 0 검증 결과

Windows의 MATLAB R2025a Update 1에서 `main` 실행과 파라미터 테스트 5개 통과를 확인했습니다. 테스트는 단위와 배열 순서, 기하 치수, 관절 축과 범위, 질량과 가정, 참조 파일 경로를 확인합니다. 첫 테스트 실행에서 MATLAB이 테스트 폴더로 이동하면서 패키지를 찾지 못했습니다. 테스트에 프로젝트 루트를 임시로 추가하도록 수정한 뒤 전체 테스트를 다시 실행했습니다.

Optimization Toolbox 경로와 라이선스를 확인했고, 독립적인 1변수 문제에서 `quadprog`가 `x = 1.00000000`, `exitflag = 1`을 반환했습니다. Phase 0 실행은 기본 MATLAB이면 됩니다.

향후 정역학 모델은 몸체 원점에 질량을 집중시키는 근사를 사용합니다. 확인한 수치와 가정은 [모델 출처](docs/model_sources.md), 단계별 범위는 [프로젝트 계획](PROJECT_PLAN.md)에 기록했습니다.
