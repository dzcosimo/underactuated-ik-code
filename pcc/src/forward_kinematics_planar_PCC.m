function x = forward_kinematics_planar_PCC(q, s, L)
%FORWARD_KINEMATICS_PLANAR_PCC  Position of a point of a planar piecewise-constant-curvature robot.
%   x = forward_kinematics_planar_PCC(q, s, L)
%     q  [k x 1] curvatures of the first k segments (body coordinates; numeric or symbolic)
%     s  normalized arc length in [0, 1] on the LAST of these k segments (s = 1: its end)
%     L  [>= k x 1] segment lengths (only the first k are used)
%   x  [2 x 1] position in the base frame. x = forward_kinematics_planar_PCC(q, 1, L) with all n
%   curvatures is the tip, i.e. the task map h(q) of the paper.
R = eye(3);
for i = 1:numel(q)
    if i < numel(q)
        rt = CC_rt_planar(1, q(i));
    else
        rt = CC_rt_planar(s, q(i));
    end
    rt(1:2, 3) = L(i)*rt(1:2, 3);
    R = R*rt;
end
x = [eye(2), zeros(2, 1)]*R*[0; 0; 1];
end
