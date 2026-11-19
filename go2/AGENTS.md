# AGENTS.md — go2 (MATLAB R2025a Static Quadruped Study)

## 1. Project purpose and boundaries

This repository is a learning-oriented **Unitree Go2 simplified-model project in MATLAB R2025a**. The end goal is to understand and numerically validate quadruped **kinematics and STATIC equilibrium** before moving to an independent MuJoCo project for dynamic standing, LQR and MPC locomotion.

**Final MATLAB deliverables:**
1. 3D visualization: rectangular torso + line-segment legs with the Go2's four 3-DoF legs.
2. FK, IK, analytic geometric leg Jacobian, joint limits and numerical verification.
3. Smooth joint-space homing animation (kinematic interpolation; NOT actuator/physics simulation).
4. Foot-anchored standing-pose computation over flat and inclined planar ground; world-level body roll/pitch targets, workspace/limit/contact checks.
5. Lumped-mass STATIC equilibrium: ground reaction force (GRF) distribution with a constrained quadratic program (QP); friction-cone approximation, contact feasibility and simplified joint torque estimates.
6. Reproducible parameter-sweep demos, plots, automated tests, clear limits and README.

**Outside MATLAB scope:** floating-base rigid-body forward dynamics; physical contact/collision simulation; torque-driven PD stabilization; LQR/MPC walking; reinforcement learning; actuator thermal/electrical models; whole-body control; manipulator integration. These belong to future MuJoCo/research projects. Don't present animation, IK or QP feasibility as proof of dynamic stability.

## 2. Incremental implementation policy

There is a planned roadmap, but **implement only the phase explicitly requested in the current Codex message**. Never build future phases without a separate user request. A request to explain the overall plan is NOT authorization to implement all phases.

- **Phase 0 — initialization and reference audit:** discover existing files/environment; fetch/inspect/pin a Unitree Go2 source model if accessible; create docs, directory skeleton, a verified `go2.model.parameters()` and basic parameter smoke tests. No FK/IK/GUI/QP here.
- **Phase 1 — kinematics & rendering:** Go2-specific transform chain and FK for all legs, 3D box/line rendering, basic tests.
- **Phase 2 — inverse/differential kinematics:** IK with feasibility and branch handling, analytic Jacobian, finite-difference and FK↔IK tests.
- **Phase 3 — homing and terrain:** joint interpolation; separate foot-locked body pose solver; planar terrain, slope and contact geometry checks; demos/tests. Remain kinematic, not dynamic.
- **Phase 4 — static GRF/QP:** exact force/moment balance constraints for the simplified model, contact normals, unilateral contact, friction-pyramid constraints, optional torque bounds and solver diagnostics; tests.
- **Phase 5 — analysis and polish:** parameter sweeps, documentation, clearer figures/UI if justified, complete regression suite, summary of limits and MuJoCo handoff.

At the end of each phase report: changed files, mathematical conventions, source provenance, commands and ACTUAL test results, limitations, and the next proposed phase (not implemented). If MATLAB cannot run, say so and give exact manual instructions; never pretend tests passed. Do not silently skip, weaken or disable failed tests.

## 3. Canonical robot sources and provenance

**Primary reference:** Unitree `unitreerobotics/unitree_mujoco`, `unitree_robots/go2/go2.xml`:
https://github.com/unitreerobotics/unitree_mujoco/blob/main/unitree_robots/go2/go2.xml

**Cross-check:** Unitree `unitreerobotics/unitree_ros`, `robots/go2_description/urdf/go2_description.urdf`:
https://github.com/unitreerobotics/unitree_ros/blob/master/robots/go2_description/urdf/go2_description.urdf

- Source files may change. Whenever possible record the **resolved Git commit SHA** and source URL in `docs/model_sources.md`, and preserve the downloaded reference in `reference/` with attribution. Do not claim a commit is pinned until verified.
- Do **not** casually blend MJCF masses, URDF masses/rotors, RL environment gains and real-hardware specifications. Label every value as one of: `reference model`, `simplifying assumption`, `example only`, or `to verify`.
- Treat any previously mentioned number as provisional until checked against the chosen source. If the source is unavailable, mark unverified rather than fabricate certainty.

Useful values seen in the official MJCF, to verify again against the pinned file:
- Hip joint origins relative to base: FL `[+0.1934,+0.0465,0]`, FR `[+0.1934,-0.0465,0]`, RL `[-0.1934,+0.0465,0]`, RR `[-0.1934,-0.0465,0]` m.
- Hip-to-thigh Y offsets: left `+0.0955`, right `-0.0955` m; thigh and calf nominal lengths: `0.213` m each.
- Joint axes: hip ab/adduction along local +X; thigh/calf about local +Y, but use the actual MJCF transform chain for signs and frames.
- Foot-frame nominal center at calf `[0,0,-0.213]` m; MJCF foot collision sphere center may instead be `[-0.002,0,-0.213]` m in calf coordinates, radius `0.022` m. Explicitly distinguish foot frame, sphere center and ground contact point.
- MJCF torso mass `6.921` kg; per leg hip `0.678`, thigh `1.152`, calf `0.241352` kg. Their total is `15.206408` kg for this model. It does NOT define the exact real-hardware mass or the exact overall center of mass.
- MJCF joint/actuator limits must be verified per joint: hip `[-1.0472,1.0472]` rad, front thigh `[-1.5708,3.4907]` rad, rear thigh `[-0.5236,4.5379]` rad, calf `[-2.7227,-0.83776]` rad; model torque control ranges hip/thigh `±23.7` Nm, calf `±45.43` Nm. These are **model control ranges**, not certified continuous motor ratings.
- Visualization-only torso box `[0.45,0.18,0.12]` m is an explicit approximation, **not an official exterior body dimension**.
- Initial mass model: all `15.206408` kg lumped at an **assumed base-frame CoM** `[0,0,0]` m. This is a simplifying assumption, NOT actual Go2 CoM. No rotational inertia is required for STATIC analysis; dynamic extension will require inertia.
- Default `mu` for terrain is an editable **assumption**, not an experimentally identified coefficient. Avoid conflating model foot friction attributes with a universally correct ground friction coefficient.
- The Go2 MJCF `base_link pos=[0,0,0.445]` is a model configuration in the file, not a proven default standing height. Derive a feasible standing height from FK and contact geometry, rather than hardcoding 0.32 or 0.445 as ground truth.

## 4. Coordinate, unit, and shape contracts

- SI units: m, kg, s, rad, N, Nm. Never silently mix degrees and radians.
- Right-handed world W and body B frames: +X forward, +Y left, +Z up; gravity in W `[0;0;-9.81]` m/s^2.
- A body rotation `R_WB` maps body-frame vectors into world frame: `p_W = p_WB + R_WB*p_B`. Use column vectors.
- Leg order for 4x3 joint matrix is always **FL, FR, RL, RR**; columns always **hip, thigh, calf**. A 3x4 foot position matrix follows the same leg order.
- Clearly label transforms, Jacobians, forces and locations with their reference frame; e.g. `footPosW`, `footPosB`, `J_foot_B`, `contactPosW`.
- State representation: `state.base.pos` as `3x1`, `state.base.R` as `3x3`, `state.q` as `4x3`. Geometry/math functions take explicit `p` and `state` and return results; no hidden globals.
- Terrain represented by unit `normalW` and `pointW`, optional a tangent basis (`t1W`,`t2W`), and `mu`. Separate definitions for foothold reference, sphere center and actual contact point.
- Prefer structs and pure functions. Base MATLAB namespace structure using `+go2` folder packages, e.g. `go2.model.parameters()`, `go2.kin.forwardRobot(...)`. Keep the root folder on the MATLAB path. Don't add nested package folders individually.

## 5. Numerical and physical correctness

- Rendering ≠ physics. Kinematic homing ≠ PD control. IK horizontal-body pose ≠ stable equilibrium.
- FK and Jacobian must be independently checked, including left/right sign and forward/back distinction; where feasible compare against the source transform hierarchy and use finite differences. Include zero configurations and non-symmetric configurations.
- IK must indicate unreachable targets, joint limit failures, singularities and alternative branches, and avoid returning NaN or a plausible-looking invalid pose as success.
- For a nonzero-radius spherical foot on a plane, the sphere center must satisfy signed distance approximately equal to the radius for contact (with a clearly defined sign and normal). Do not simply set the foot-frame origin `z=0`.
- On slope changes, you cannot retain four arbitrary previous world footholds and also assume they lie on the new plane. Define explicit scenarios: (a) generate new valid footholds on the plane, then lock them, or (b) preserve existing valid footholds on an unchanged plane while moving the body. No silent repositioning during foot-locked motion.
- Quasistatic equilibrium for contacts `c_i` and concentrated CoM `r_G`:
  `sum_i f_i + m*g = 0` and `sum_i (c_i-r_G) x f_i = 0`.
- Each contact force is the **ground-on-robot** force. Require compressive contact `n_i' * f_i >= 0`, with a conservative friction pyramid, e.g. `abs(t_j' * f_i) <= (mu/sqrt(2))*(n_i' * f_i)` for j=1,2.
- Different contacts must use documented and consistent local normals/tangent bases. Swing legs have `f_i=0` (a future extension; standing initially has four contacts).
- Use a clear QP objective (e.g. weighted contact-force norm or distance to reference) and enforce **force/moment equilibrium as hard equality constraints**; report infeasibility rather than disguise it with only soft penalties.
- Validate the returned forces independently via residuals, inequality violations and solver exit status. Check torque bounds if modeled. Support-polygon heuristic alone must not be called proof of full contact feasibility.
- Joint torque from foot contact force depends on Jacobian reference frame and on whether the force is ground-on-robot or robot-on-ground. Derive and document the sign using virtual work / static leg equilibrium; do not casually write `tau=J'*f` without convention. Lumped-mass/massless-leg approximation omits link gravity and rotor effects.
- Do not invent physically justified threshold values: document numerical tolerances as test tolerances and explain their scale.

## 6. Planned repository layout (create files when needed)

```
go2/
  AGENTS.md
  README.md
  PROJECT_PLAN.md
  main.m
  run_tests.m
  +go2/
    +model/parameters.m, initialState.m
    +math/rotX.m, rotY.m, skew.m              [optional as needed]
    +kin/forwardLeg.m, forwardRobot.m, inverseLeg.m, jacobianLeg.m
    +motion/homingTrajectory.m, solveStandingPose.m
    +terrain/createPlane.m, tangentBasis.m
    +statics/supportPolygon.m, solveGRF.m, contactTorques.m, checkEquilibrium.m
    +viz/drawScene.m, animateMotion.m, plotAnalysis.m
  demos/demo01_kinematics.m, demo02_homing.m, demo03_slope.m, demo04_grf_qp.m
  tests/test_parameters.m, test_kinematics.m, test_jacobian.m,
        test_standing.m, test_statics.m
  docs/model_sources.md
  reference/go2.xml (only if legitimately fetched and attributed)
  results/
```

This is a target architecture, not permission to generate dozens of empty placeholder `.m` files. Add executable files only when their phase is implemented. Maintain a single authoritative parameter function; avoid copied constants across code.

## 7. MATLAB R2025a execution and testing

- First inspect what MATLAB and toolboxes are actually accessible. `matlab -batch "disp(version); ver"` is suitable if executable is on PATH. `which('quadprog')` in MATLAB checks solver presence. Don't claim local execution when unavailable.
- Base MATLAB should suffice for Phase 0–3; use optional Robotics System Toolbox only for reference verification, not as an unexpected hard dependency. Optimization Toolbox / `quadprog` is for Phase 4; document absence and propose an explicit fallback rather than changing the mathematical problem silently.
- Create tests with MATLAB `matlab.unittest` or function-based `functiontests`. Provide a reproducible `run_tests.m`, run via GUI or `matlab -batch "run_tests"` from repository root when possible.
- When changing a numerical routine, run the directly relevant tests and record pass/fail. Don't modify test expectations to conceal implementation errors.
- MATLAB files: one clear public function per file, matching filename; descriptive names and comments for units/frames/shapes; no `eval`, hidden base-workspace dependencies, or gratuitous classes; no huge monolithic `main.m`.
- Keep figures deterministic with `axis equal`, labeled W axes and leg labels. Headless tests must not require GUI rendering.
- Log actual output: values, tolerances, failures, solver status, sources, and unknowns. Never present illustrative numbers as measured experimental results.

## 8. Communication, review, and scope control

- Explain work and results in Korean, using English identifiers for code and variables.
- For each phase, provide: (a) purpose; (b) mathematical model and assumptions; (c) changed file paths; (d) instructions to run; (e) actual tests and results; (f) known limitations; (g) unimplemented next step.
- Be cautious with numerical claims and sources. Include links and a source revision when asserting model-specific numbers.
- Before a breaking API change, inspect existing callers and tests; do not overwrite user work without checking. Respect local repository changes and don't silently overwrite existing `AGENTS.md`.
- Do not change project goals or silently add MuJoCo, RL, LQR, MPC, Simscape, Simulink or other large dependencies during the MATLAB phases.
- Only after the user explicitly authorizes the next phase should you implement it.
