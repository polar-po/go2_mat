function fig = demo01_kinematics(p)
%DEMO01_KINEMATICS Compare the home pose to a static FL joint-angle edit.

arguments
    p (1,1) struct = go2.model.parameters()
end

homeState = go2.model.initialState(p);
homeRobot = go2.kin.forwardRobot(p, homeState);

editedState = homeState;
% With the body held fixed, this edit lifts the front-left foot.
editedState.q(1, :) = homeState.q(1, :) + [0.25, -0.10, -0.40];
editedRobot = go2.kin.forwardRobot(p, editedState);

fprintf('\nPhase 1 | Forward kinematics\n');
fprintf('Home body height: %.6f m\n', homeState.base.pos(3));
fprintf('Joint order (rad): hip, thigh, calf\n');
fprintf('FL home angles:   [% .6f, % .6f, % .6f]\n', homeState.q(1, :));
fprintf('FL edited angles: [% .6f, % .6f, % .6f]\n', editedState.q(1, :));
fprintf('\nWorld-frame foot reference positions [m]\n');
fprintf('Leg      Home X      Home Y      Home Z      Edit X      Edit Y      Edit Z\n');
for legIndex = 1:4
    fprintf('%s     % .6f   % .6f   % .6f   % .6f   % .6f   % .6f\n', ...
        p.legOrder(legIndex), homeRobot.footPosW(:, legIndex), ...
        editedRobot.footPosW(:, legIndex));
end
fprintf('FL foot displacement [m]: [% .6f, % .6f, % .6f]\n', ...
    editedRobot.footPosW(:, 1) - homeRobot.footPosW(:, 1));
fprintf('Home foot-sphere clearance [m], FL FR RL RR: [% .3e, % .3e, % .3e, % .3e]\n', ...
    homeRobot.footSphereCenterW(3, :) - p.geometry.footSphereRadius);
fprintf('Edited FL foot-sphere clearance [m]: %.6f\n', ...
    editedRobot.footSphereCenterW(3, 1) - p.geometry.footSphereRadius);

fig = figure('Name', 'Go2 Forward Kinematics', 'NumberTitle', 'off', ...
    'Color', 'w', 'Position', [100, 100, 1050, 520]);
tiles = tiledlayout(fig, 1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
homeAxes = nexttile(tiles);
editedAxes = nexttile(tiles);
go2.viz.drawScene(homeAxes, p, homeState.base, homeRobot);
go2.viz.drawScene(editedAxes, p, editedState.base, editedRobot);
title(homeAxes, 'Home joint angles');
title(editedAxes, 'Front-left joint angles edited');
linkaxes([homeAxes, editedAxes], 'xyz');
end
