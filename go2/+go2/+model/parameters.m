function p = parameters()
%PARAMETERS Unitree Go2 reference geometry and MATLAB study assumptions.
% Offsets are 3-by-4 with columns FL, FR, RL, RR. Joint bounds are
% 4-by-3 with rows in the same leg order and columns hip, thigh, calf.

p.source.commit = '1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d';
p.source.url = ['https://github.com/unitreerobotics/unitree_mujoco/blob/' ...
    p.source.commit '/unitree_robots/go2/go2.xml'];
p.source.localFile = 'reference/go2.xml';

p.units = struct('length', "m", 'mass', "kg", 'time', "s", ...
    'angle', "rad", 'force', "N", 'torque', "N*m", 'acceleration', "m/s^2");
p.legOrder = ["FL", "FR", "RL", "RR"];
p.jointOrder = ["hip", "thigh", "calf"];
p.frameAxes = ["forward", "left", "up"];

% Reference model. Each offset is expressed in the parent link frame.
% The joint origins coincide with their link origins. At zero angle the
% child link axes are aligned with the parent link axes.
p.geometry.hipOriginB = [0.1934, 0.1934, -0.1934, -0.1934;
                        0.0465, -0.0465, 0.0465, -0.0465;
                        0, 0, 0, 0];
p.geometry.thighOriginHip = [0, 0, 0, 0;
                             0.0955, -0.0955, 0.0955, -0.0955;
                             0, 0, 0, 0];
p.geometry.calfOriginThigh = [0; 0; -0.213];
p.geometry.footOriginCalf = [0; 0; -0.213];
p.geometry.footSphereCenterCalf = [-0.002; 0; -0.213];
p.geometry.footSphereRadius = 0.022;

% Axes are local unit vectors, one column per joint.
p.joints.axisLocal = [1, 0, 0; 0, 1, 1; 0, 0, 0];
p.joints.lower = [-1.0472, -1.5708, -2.7227;
                  -1.0472, -1.5708, -2.7227;
                  -1.0472, -0.5236, -2.7227;
                  -1.0472, -0.5236, -2.7227];
p.joints.upper = [1.0472, 3.4907, -0.83776;
                  1.0472, 3.4907, -0.83776;
                  1.0472, 4.5379, -0.83776;
                  1.0472, 4.5379, -0.83776];
% Symmetric MJCF motor control ranges, with model transmission gear 1.
p.joints.motorControlAbsMax = [23.7, 23.7, 45.43];

% Joint angles (rad) from the MJCF home keyframe. Base pose is set separately.
p.referencePose.homeJointAngles = repmat([0, 0.9, -1.8], numel(p.legOrder), 1);

p.mass.torso = 6.921;
p.mass.linksPerLeg = [0.678, 1.152, 0.241352];
p.mass.total = p.mass.torso + numel(p.legOrder) * sum(p.mass.linksPerLeg);

% Simplifying assumptions and an editable friction example.
p.assumptions.lumpedCoMB = [0; 0; 0];
p.assumptions.torsoBoxSize = [0.45; 0.18; 0.12];
p.assumptions.gravityW = [0; 0; -9.81];
p.examples.terrainMu = 0.5;
end
