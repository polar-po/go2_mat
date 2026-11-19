function state = initialState(p)
%INITIALSTATE Level home pose with four foot spheres tangent to z_W = 0.
% Keep the MJCF home joint angles and derive the body height from geometry.

state.q = p.referencePose.homeJointAngles;
state.base.pos = zeros(3, 1);
state.base.R = eye(3);
robot = go2.kin.forwardRobot(p, state);
centerHeights = robot.footSphereCenterB(3, :);

% A single vertical translation can place all four equal-radius spheres on
% the floor only when their centres start at the same height.
heightTolerance = 1e-12; % Metres, for roundoff in the symmetric pose.
if max(centerHeights) - min(centerHeights) > heightTolerance
    error('go2:model:UnequalHomeFootHeights', ...
        'The level home pose must have four foot spheres at equal heights.');
end
state.base.pos(3) = p.geometry.footSphereRadius - min(centerHeights);
end
