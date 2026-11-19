function robot = forwardRobot(p, state)
%FORWARDROBOT Compute four-leg FK in body B and world W coordinates.
% state.q is 4-by-3 (FL, FR, RL, RR by hip, thigh, calf), in radians.
% state.base.pos is the 3-by-1 body origin in W, in metres.
% state.base.R is the 3-by-3 rotation R_WB, mapping B vectors into W.
% W and B use +X forward, +Y left, +Z up at the neutral body pose.
% jointPosB/W are 3-by-3-by-4 (xyz by joint by leg).
% footPosB/W and footSphereCenterB/W are 3-by-4 (xyz by leg).

q = state.q;
basePosW = state.base.pos;
R_WB = state.base.R;
validateattributes(q, {'double'}, {'size', [4, 3], 'real', 'finite'});
validateattributes(basePosW, {'double'}, {'size', [3, 1], 'real', 'finite'});
validateattributes(R_WB, {'double'}, {'size', [3, 3], 'real', 'finite'});

rotationTolerance = 1e-10; % Dimensionless tolerance for a double-precision rotation.
if norm(R_WB.' * R_WB - eye(3), 'fro') > rotationTolerance || det(R_WB) <= 0
    error('go2:kin:InvalidBaseRotation', 'state.base.R must be a proper rotation matrix.');
end

robot.jointPosB = zeros(3, 3, 4);
robot.jointPosW = zeros(3, 3, 4);
robot.footPosB = zeros(3, 4);
robot.footSphereCenterB = zeros(3, 4);

for legIndex = 1:4
    leg = go2.kin.forwardLeg(p, legIndex, q(legIndex, :));
    robot.jointPosB(:, :, legIndex) = leg.jointPosB;
    robot.jointPosW(:, :, legIndex) = basePosW + R_WB * leg.jointPosB;
    robot.footPosB(:, legIndex) = leg.footPosB;
    robot.footSphereCenterB(:, legIndex) = leg.footSphereCenterB;
end

robot.footPosW = basePosW + R_WB * robot.footPosB;
robot.footSphereCenterW = basePosW + R_WB * robot.footSphereCenterB;
end
