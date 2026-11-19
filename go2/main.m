function main(showDemo)
%MAIN Print the Go2 model summary and open the Phase 1 FK demo.
% main(false) prints the Phase 0 summary on its own.

arguments
    showDemo (1,1) logical = true
end

p = go2.model.parameters();
fprintf('Unitree Go2 | MATLAB Phase 0\n');
fprintf('MJCF commit: %s\n', p.source.commit);
fprintf('Leg order: %s\n', strjoin(p.legOrder, ', '));
fprintf('Joint order: %s\n', strjoin(p.jointOrder, ', '));
fprintf('Thigh / calf length: %.3f / %.3f m\n', ...
    norm(p.geometry.calfOriginThigh), norm(p.geometry.footOriginCalf));
fprintf('Foot sphere radius: %.3f m\n', p.geometry.footSphereRadius);
fprintf('Total reference mass: %.6f kg\n', p.mass.total);
fprintf('Assumed CoM in B: [%.3f, %.3f, %.3f] m\n', p.assumptions.lumpedCoMB);
fprintf('Example terrain friction coefficient: %.2f\n', p.examples.terrainMu);

if showDemo
    projectRoot = fileparts(mfilename('fullpath'));
    originalPath = path;
    restorePath = onCleanup(@() path(originalPath));
    addpath(fullfile(projectRoot, 'demos'));
    demo01_kinematics(p);
end
end
