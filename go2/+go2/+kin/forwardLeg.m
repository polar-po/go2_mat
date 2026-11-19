function leg = forwardLeg(p, legIndex, q)
%FORWARDLEG Compute Go2 leg positions in the body frame B.
% legIndex is 1=FL, 2=FR, 3=RL, 4=RR. q = [hip, thigh, calf] in rad.
% jointPosB is 3-by-3 with hip, thigh, calf columns. footPosB and
% footSphereCenterB are distinct 3-by-1 points. All positions use metres.
% FK also supports angles outside joint limits for geometry checks.

arguments
    p (1,1) struct
    legIndex (1,1) double {mustBeInteger, mustBeInRange(legIndex, 1, 4)}
    q (1,3) double {mustBeReal, mustBeFinite}
end

axesLocal = p.joints.axisLocal;
geometry = p.geometry;

% An offset is translated in the parent frame before the child rotates.
hipPosB = geometry.hipOriginB(:, legIndex);
R_BHip = axisRotation(axesLocal(:, 1), q(1));
thighPosB = hipPosB + R_BHip * geometry.thighOriginHip(:, legIndex);
R_BThigh = R_BHip * axisRotation(axesLocal(:, 2), q(2));
calfPosB = thighPosB + R_BThigh * geometry.calfOriginThigh;
R_BCalf = R_BThigh * axisRotation(axesLocal(:, 3), q(3));

leg.jointPosB = [hipPosB, thighPosB, calfPosB];
leg.footPosB = calfPosB + R_BCalf * geometry.footOriginCalf;
leg.footSphereCenterB = calfPosB + R_BCalf * geometry.footSphereCenterCalf;
end

function R = axisRotation(axis, angle)
% Rodrigues' formula for a right-handed rotation around a local unit axis.
x = axis(1);
y = axis(2);
z = axis(3);
crossMatrix = [0, -z, y; z, 0, -x; -y, x, 0];
c = cos(angle);
R = c * eye(3) + (1 - c) * (axis * axis.') + sin(angle) * crossMatrix;
end
