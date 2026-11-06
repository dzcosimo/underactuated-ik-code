function [bb, ee] = robot_backbone(q, L)
%ROBOT_BACKBONE  Points of the robot backbone for drawing.
%   [bb, ee] = robot_backbone(q, L)
%     q   [n x 1] curvatures, L [n x 1] segment lengths
%     bb  [2 x (20 n + 1)] backbone samples from the base (20 per CC segment)
%     ee  [2 x (n + 1)] base and segment ends; ee(:, ceil(n/2) + 1) is the end of the first
%         actuated section, ee(:, end) the tip
n = numel(q);
NP = 20;
bb = zeros(2, n*NP + 1);
ee = zeros(2, n + 1);
for i = 1:n
    for k = 1:NP
        bb(:, (i - 1)*NP + k + 1) = forward_kinematics_planar_PCC(q(1:i), k/NP, L);
    end
    ee(:, i + 1) = forward_kinematics_planar_PCC(q(1:i), 1, L);
end
end
