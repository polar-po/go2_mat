function tests = test_kinematics
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
projectRoot = fileparts(fileparts(mfilename('fullpath')));
testCase.applyFixture(matlab.unittest.fixtures.PathFixture(projectRoot));
testCase.TestData.p = go2.model.parameters();
% Absolute tolerance in metres for roundoff in the rigid transforms.
testCase.TestData.positionTolerance = 1e-12;
end

function testFourLegOutputShapesAndFinitePositions(testCase)
p = testCase.TestData.p;
robot = go2.kin.forwardRobot(p, asymmetricState());

verifySize(testCase, robot.jointPosB, [3, 3, 4]);
verifySize(testCase, robot.jointPosW, [3, 3, 4]);
verifySize(testCase, robot.footPosB, [3, 4]);
verifySize(testCase, robot.footPosW, [3, 4]);
verifySize(testCase, robot.footSphereCenterB, [3, 4]);
verifySize(testCase, robot.footSphereCenterW, [3, 4]);
positions = [robot.jointPosB(:); robot.jointPosW(:); ...
    robot.footPosB(:); robot.footPosW(:); ...
    robot.footSphereCenterB(:); robot.footSphereCenterW(:)];
verifyTrue(testCase, all(isfinite(positions)) && isreal(positions));
end

function testZeroAnglesAgainstMjcfLandmarks(testCase)
p = testCase.TestData.p;
tol = testCase.TestData.positionTolerance;
state = asymmetricState();
state.q = zeros(4, 3);
robot = go2.kin.forwardRobot(p, state);

% Landmarks read from the pinned MJCF hierarchy with no rotations.
hip = [0.1934, 0.1934, -0.1934, -0.1934;
       0.0465, -0.0465, 0.0465, -0.0465; zeros(1, 4)];
thigh = [hip(1, :); 0.142, -0.142, 0.142, -0.142; zeros(1, 4)];
calf = [thigh(1:2, :); repmat(-0.213, 1, 4)];
foot = [thigh(1:2, :); repmat(-0.426, 1, 4)];
sphereCenter = [0.1914, 0.1914, -0.1954, -0.1954; foot(2:3, :)];

verifyEqual(testCase, reshape(robot.jointPosB(:, 1, :), 3, 4), hip, 'AbsTol', tol);
verifyEqual(testCase, reshape(robot.jointPosB(:, 2, :), 3, 4), thigh, 'AbsTol', tol);
verifyEqual(testCase, reshape(robot.jointPosB(:, 3, :), 3, 4), calf, 'AbsTol', tol);
verifyEqual(testCase, robot.footPosB, foot, 'AbsTol', tol);
verifyEqual(testCase, robot.footSphereCenterB, sphereCenter, 'AbsTol', tol);
end

function testHipRollSignsOnLeftAndRight(testCase)
p = testCase.TestData.p;
tol = testCase.TestData.positionTolerance;
fl = go2.kin.forwardLeg(p, 1, [pi/2, 0, 0]);
fr = go2.kin.forwardLeg(p, 2, [pi/2, 0, 0]);

% A +90 deg X turn maps +Y to +Z and -Z to +Y on both legs.
verifyEqual(testCase, fl.jointPosB(:, 2), [0.1934; 0.0465; 0.0955], 'AbsTol', tol);
verifyEqual(testCase, fr.jointPosB(:, 2), [0.1934; -0.0465; -0.0955], 'AbsTol', tol);
verifyEqual(testCase, fl.jointPosB(:, 3), [0.1934; 0.2595; 0.0955], 'AbsTol', tol);
verifyEqual(testCase, fr.jointPosB(:, 3), [0.1934; 0.1665; -0.0955], 'AbsTol', tol);
end

function testMirroredLegsAtNonzeroAngles(testCase)
p = testCase.TestData.p;
tol = testCase.TestData.positionTolerance;
state = asymmetricState();
state.q = [0.21, 0.8, -1.6; -0.21, 0.8, -1.6;
           -0.13, 1.0, -1.9; 0.13, 1.0, -1.9];
robot = go2.kin.forwardRobot(p, state);
mirrorAcrossXZ = [1; -1; 1];

verifyEqual(testCase, robot.footPosB(:, [2, 4]), ...
    mirrorAcrossXZ .* robot.footPosB(:, [1, 3]), 'AbsTol', tol);
verifyEqual(testCase, robot.footSphereCenterB(:, [2, 4]), ...
    mirrorAcrossXZ .* robot.footSphereCenterB(:, [1, 3]), 'AbsTol', tol);
end

function testWorldTranslationMovesEveryPoint(testCase)
p = testCase.TestData.p;
tol = testCase.TestData.positionTolerance;
state = asymmetricState();
original = go2.kin.forwardRobot(p, state);
shift = [0.23; -0.71; 0.39];
state.base.pos = shift;
moved = go2.kin.forwardRobot(p, state);

verifyEqual(testCase, moved.jointPosB, original.jointPosB);
verifyEqual(testCase, moved.footPosB, original.footPosB);
verifyEqual(testCase, moved.footSphereCenterB, original.footSphereCenterB);
verifyEqual(testCase, moved.jointPosW - original.jointPosW, ...
    repmat(shift, 1, 3, 4), 'AbsTol', tol);
verifyEqual(testCase, moved.footPosW - original.footPosW, repmat(shift, 1, 4), 'AbsTol', tol);
verifyEqual(testCase, moved.footSphereCenterW - original.footSphereCenterW, ...
    repmat(shift, 1, 4), 'AbsTol', tol);
end

function testWorldRotationByIndependentCoordinateMapping(testCase)
p = testCase.TestData.p;
tol = testCase.TestData.positionTolerance;
state = asymmetricState();
original = go2.kin.forwardRobot(p, state);
state.base.pos = [0.3; -0.2; 0.5];
state.base.R = [0, 0, 1; 1, 0, 0; 0, 1, 0];
rotated = go2.kin.forwardRobot(p, state);

% This proper rotation sends B coordinates (x,y,z) to W (z,x,y).
offset = state.base.pos;
verifyEqual(testCase, rotated.jointPosB, original.jointPosB);
verifyEqual(testCase, rotated.jointPosW, ...
    original.jointPosB([3, 1, 2], :, :) + offset, 'AbsTol', tol);
verifyEqual(testCase, rotated.footPosW, ...
    original.footPosB([3, 1, 2], :) + offset, 'AbsTol', tol);
verifyEqual(testCase, rotated.footSphereCenterW, ...
    original.footSphereCenterB([3, 1, 2], :) + offset, 'AbsTol', tol);
end

function testEachPitchJointUsesItsOwnPivot(testCase)
p = testCase.TestData.p;
tol = testCase.TestData.positionTolerance;
zero = go2.kin.forwardLeg(p, 1, zeros(1, 3));
thigh = go2.kin.forwardLeg(p, 1, [0, pi/2, 0]);
calf = go2.kin.forwardLeg(p, 1, [0, 0, -pi/2]);

% A +90 deg thigh turn maps both down-links to -X.
verifyEqual(testCase, thigh.jointPosB(:, 1:2), zero.jointPosB(:, 1:2));
verifyEqual(testCase, thigh.jointPosB(:, 3), [-0.0196; 0.142; 0], 'AbsTol', tol);
verifyEqual(testCase, thigh.footPosB, [-0.2326; 0.142; 0], 'AbsTol', tol);
% A -90 deg calf turn leaves the knee fixed and sends only the foot toward +X.
verifyEqual(testCase, calf.jointPosB, zero.jointPosB);
verifyEqual(testCase, calf.footPosB, [0.4064; 0.142; -0.213], 'AbsTol', tol);
end

function testRotatedChildAxesFollowParent(testCase)
p = testCase.TestData.p;
tol = testCase.TestData.positionTolerance;
leg = go2.kin.forwardLeg(p, 1, [pi/2, pi/2, -pi/2]);

% Hip roll raises the thigh pivot. Thigh pitch sends the knee toward -X.
% The two pitches cancel for the foot link, so the hip roll sends it along +Y.
verifyEqual(testCase, leg.jointPosB(:, 3), [-0.0196; 0.0465; 0.0955], 'AbsTol', tol);
verifyEqual(testCase, leg.footPosB, [-0.0196; 0.2595; 0.0955], 'AbsTol', tol);
verifyEqual(testCase, leg.footSphereCenterB, [-0.0216; 0.2595; 0.0955], 'AbsTol', tol);
end

function testLinkLengthsAtAsymmetricAngles(testCase)
p = testCase.TestData.p;
tol = testCase.TestData.positionTolerance;
robot = go2.kin.forwardRobot(p, asymmetricState());
for legIndex = 1:4
    points = [robot.jointPosW(:, :, legIndex), robot.footPosW(:, legIndex)];
    verifyEqual(testCase, vecnorm(diff(points, 1, 2)), [0.0955, 0.213, 0.213], 'AbsTol', tol);
end
verifyEqual(testCase, vecnorm(robot.footSphereCenterW - robot.footPosW), ...
    repmat(0.002, 1, 4), 'AbsTol', tol);
end

function testHomeGeometryTouchesFlatGround(testCase)
p = testCase.TestData.p;
tol = testCase.TestData.positionTolerance;
state = go2.model.initialState(p);
robot = go2.kin.forwardRobot(p, state);

% Home pitches are +0.9 and -0.9 relative to vertical for the two links.
expectedHeight = 0.022 + 0.426*cos(0.9) + 0.002*sin(0.9);
verifyEqual(testCase, state.q, repmat([0, 0.9, -1.8], 4, 1));
verifyEqual(testCase, state.base.pos, [0; 0; expectedHeight], 'AbsTol', tol);
verifyEqual(testCase, state.base.R, eye(3));
verifyGreaterThanOrEqual(testCase, state.q, p.joints.lower);
verifyLessThanOrEqual(testCase, state.q, p.joints.upper);
verifyEqual(testCase, robot.footSphereCenterW(3, :), repmat(0.022, 1, 4), 'AbsTol', tol);
end

function testWorldRotationRejectsReflection(testCase)
p = testCase.TestData.p;
state = asymmetricState();
state.base.R = diag([1, -1, 1]);
verifyError(testCase, @() go2.kin.forwardRobot(p, state), 'go2:kin:InvalidBaseRotation');
end

function state = asymmetricState()
state.base.pos = zeros(3, 1);
state.base.R = eye(3);
state.q = [0.10, 0.85, -1.62;
          -0.23, 0.94, -1.78;
           0.18, 1.11, -2.15;
          -0.31, 1.23, -1.25];
end
