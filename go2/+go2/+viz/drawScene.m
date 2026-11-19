function drawScene(ax, p, base, robot)
%DRAWSCENE Draw a Go2 FK result on a world-frame axes with a flat floor.
% base is state.base used for FK. robot is the result of forwardRobot.

cla(ax);
hold(ax, 'on');

% Box vertices are a visualization approximation in the body frame.
halfSize = p.assumptions.torsoBoxSize / 2;
boxSigns = [-1, 1, 1, -1, -1, 1, 1, -1;
            -1, -1, 1, 1, -1, -1, 1, 1;
            -1, -1, -1, -1, 1, 1, 1, 1];
boxW = base.pos + base.R * (halfSize .* boxSigns);
boxFaces = [1, 2, 3, 4; 5, 6, 7, 8; 1, 2, 6, 5;
            2, 3, 7, 6; 3, 4, 8, 7; 4, 1, 5, 8];

scenePoints = [boxW, reshape(robot.jointPosW, 3, []), ...
    robot.footPosW, robot.footSphereCenterW];
floorMin = min(scenePoints(1:2, :), [], 2) - 0.10;
floorMax = max(scenePoints(1:2, :), [], 2) + 0.10;
patch(ax, [floorMin(1), floorMax(1), floorMax(1), floorMin(1)], ...
    [floorMin(2), floorMin(2), floorMax(2), floorMax(2)], zeros(1, 4), ...
    [0.89, 0.91, 0.92], 'EdgeColor', [0.70, 0.73, 0.75]);
patch(ax, 'Vertices', boxW.', 'Faces', boxFaces, ...
    'FaceColor', [0.78, 0.82, 0.87], 'FaceAlpha', 0.65, ...
    'EdgeColor', [0.29, 0.34, 0.41]);

legColors = [0.12, 0.37, 0.67; 0.78, 0.31, 0.13;
             0.11, 0.50, 0.36; 0.53, 0.29, 0.62];
legLines = gobjects(1, 4);
[sphereX, sphereY, sphereZ] = sphere(12);
radius = p.geometry.footSphereRadius;
for legIndex = 1:4
    joints = robot.jointPosW(:, :, legIndex);
    points = [joints, robot.footPosW(:, legIndex)];
    color = legColors(legIndex, :);
    legLines(legIndex) = plot3(ax, points(1, :), points(2, :), points(3, :), ...
        'Color', color, 'LineWidth', 2);
    plot3(ax, joints(1, :), joints(2, :), joints(3, :), ...
        'LineStyle', 'none', 'Marker', '.', 'MarkerSize', 16, 'Color', color);
    center = robot.footSphereCenterW(:, legIndex);
    surf(ax, center(1) + radius * sphereX, center(2) + radius * sphereY, ...
        center(3) + radius * sphereZ, 'FaceColor', color, 'EdgeColor', 'none');
end

xlabel(ax, 'X forward [m]');
ylabel(ax, 'Y left [m]');
zlabel(ax, 'Z up [m]');
axis(ax, 'equal');
axis(ax, 'vis3d');
view(ax, 38, 26);
grid(ax, 'on');
legend(ax, legLines, p.legOrder, 'Location', 'northwest');
hold(ax, 'off');
end
