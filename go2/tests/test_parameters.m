function tests = test_parameters
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
projectRoot = fileparts(fileparts(mfilename('fullpath')));
testCase.applyFixture(matlab.unittest.fixtures.PathFixture(projectRoot));
end

function testOrdersAndUnits(testCase)
p = go2.model.parameters();
verifyEqual(testCase, p.legOrder, ["FL", "FR", "RL", "RR"]);
verifyEqual(testCase, p.jointOrder, ["hip", "thigh", "calf"]);
verifyEqual(testCase, p.frameAxes, ["forward", "left", "up"]);
verifyEqual(testCase, p.units, struct('length', "m", 'mass', "kg", ...
    'time', "s", 'angle', "rad", 'force', "N", 'torque', "N*m", ...
    'acceleration', "m/s^2"));
end

function testGeometry(testCase)
p = go2.model.parameters();
verifySize(testCase, p.geometry.hipOriginB, [3, 4]);
verifySize(testCase, p.geometry.thighOriginHip, [3, 4]);
verifyEqual(testCase, p.geometry.hipOriginB, ...
    [0.1934, 0.1934, -0.1934, -0.1934;
     0.0465, -0.0465, 0.0465, -0.0465;
     0, 0, 0, 0]);
verifyEqual(testCase, p.geometry.thighOriginHip, ...
    [0, 0, 0, 0; 0.0955, -0.0955, 0.0955, -0.0955; 0, 0, 0, 0]);
verifyEqual(testCase, p.geometry.calfOriginThigh, [0; 0; -0.213]);
verifyEqual(testCase, p.geometry.footOriginCalf, [0; 0; -0.213]);
verifyEqual(testCase, p.geometry.footSphereCenterCalf, [-0.002; 0; -0.213]);
verifyEqual(testCase, p.geometry.footSphereRadius, 0.022);
end

function testJointAxesAndLimits(testCase)
p = go2.model.parameters();
verifyEqual(testCase, p.joints.axisLocal, [1, 0, 0; 0, 1, 1; 0, 0, 0]);
verifySize(testCase, p.joints.lower, [4, 3]);
verifySize(testCase, p.joints.upper, [4, 3]);
verifyLessThan(testCase, p.joints.lower, p.joints.upper);
verifyEqual(testCase, p.joints.lower(:, 1), repmat(-1.0472, 4, 1));
verifyEqual(testCase, p.joints.upper(:, 1), repmat(1.0472, 4, 1));
verifyEqual(testCase, p.joints.lower(:, 2), [-1.5708; -1.5708; -0.5236; -0.5236]);
verifyEqual(testCase, p.joints.upper(:, 2), [3.4907; 3.4907; 4.5379; 4.5379]);
verifyEqual(testCase, p.joints.lower(:, 3), repmat(-2.7227, 4, 1));
verifyEqual(testCase, p.joints.upper(:, 3), repmat(-0.83776, 4, 1));
verifyEqual(testCase, p.joints.motorControlAbsMax, [23.7, 23.7, 45.43]);
end

function testMassAndStudyAssumptions(testCase)
p = go2.model.parameters();
verifyEqual(testCase, p.mass.torso, 6.921);
verifyEqual(testCase, p.mass.linksPerLeg, [0.678, 1.152, 0.241352]);
% The sum can incur binary floating-point rounding. Tolerance is in kg.
verifyEqual(testCase, p.mass.total, 15.206408, 'AbsTol', 1e-12);
verifyEqual(testCase, p.assumptions.lumpedCoMB, [0; 0; 0]);
verifyGreaterThan(testCase, p.assumptions.torsoBoxSize, zeros(3, 1));
verifyEqual(testCase, p.assumptions.gravityW, [0; 0; -9.81]);
verifyGreaterThan(testCase, p.examples.terrainMu, 0);
end

function testPinnedReference(testCase)
p = go2.model.parameters();
projectRoot = fileparts(fileparts(mfilename('fullpath')));
verifyNotEmpty(testCase, regexp(p.source.commit, '^[a-f0-9]{40}$', 'once'));
verifySubstring(testCase, p.source.url, ['/blob/' p.source.commit '/']);
verifyTrue(testCase, isfile(fullfile(projectRoot, p.source.localFile)));
end
