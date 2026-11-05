%GEN_SYMBOLIC  Symbolic model of the elastic-pointe ballerina -> model/bal_*.m (requires the
%   Symbolic Math Toolbox). The generated files are shipped, so this is needed only after a change
%   of bal_params; run_all_ballerina calls it automatically if model/bal_M.m is missing.
%
%   Generated functions (q = (qa1, qa2, qa3, qu), dq its velocity, k pointe stiffness):
%     bal_M(q)       mass matrix M(q) (4x4)
%     bal_c(q,dq)    Coriolis/centrifugal vector c = C(q,dq) dq = Mdot dq - dT/dq
%     bal_G(q,k)     gravity + spring torques G = dU/dq, U = g sum m_i z_i + k qu^2/2
%     bal_H(q,k)     potential Hessian H = d2U/dq2
%     bal_com(q)     CoM h(q) = (x_c, z_c), the task
%     bal_Jcom(q)    CoM Jacobian dh/dq = [Ja Ju] (2x4)
%     bal_points(q)  drawn points (2x6): pointe, hip, shoulder, head centre, foot, hand
%   Kinematics: segment directions e(th) = [sin th; cos th] (angles clockwise from vertical);
%   absolute angles: stance leg qu, torso qu+qa1, working leg qu+qa2 (pointing down from the
%   hip), arm qu+qa1+qa3 (pointing down from the shoulder). Uniform rods, point-mass head.
%   Checked identities: G_u = k qu - M g x_c, H_uu = k - M g z_c.

here = fileparts(mfilename('fullpath')); out = here;
addpath(fullfile(here, '..', 'src'));                   % bal_params
p = bal_params();
syms qa1 qa2 qa3 qu dqa1 dqa2 dqa3 dqu k real
q = [qa1; qa2; qa3; qu]; dq = [dqa1; dqa2; dqa3; dqu];
e = @(th) [sin(th); cos(th)];
% absolute angles and points
th_s = qu; th_t = qu + qa1; th_w = qu + qa2; th_a = qu + qa1 + qa3;
hip = p.Ls*e(th_s); sh = hip + p.Lt*e(th_t); head = sh + p.rh*e(th_t);
foot = hip - p.Lw*e(th_w); hand = sh - p.La*e(th_a);
% centres of mass of the bodies (rod midpoints, head point mass), masses, central inertias
c_s = p.Ls/2*e(th_s); c_t = hip + p.Lt/2*e(th_t); c_w = hip - p.Lw/2*e(th_w); c_a = sh - p.La/2*e(th_a);
C = [c_s c_t c_w c_a head]; m = [p.ms p.mt p.mw p.ma p.mh];
I = [p.ms*p.Ls^2 p.mt*p.Lt^2 p.mw*p.Lw^2 p.ma*p.La^2 0]/12;
w = [dqu; dqu+dqa1; dqu+dqa2; dqu+dqa1+dqa3; 0];       % absolute angular velocities
% kinetic and potential energy
T = 0; U = 0;
for i = 1:5
    v = jacobian(C(:,i), q)*dq;
    T = T + m(i)/2*(v.'*v) + I(i)/2*w(i)^2;
    U = U + m(i)*p.g*C(2,i);
end
U = U + k/2*qu^2;                                       % pointe spring
com = C*m.'/p.M;
Mm = simplify(hessian(T, dq));
cv = simplify(jacobian(Mm*dq, q)*dq - jacobian(T, q).');   % = Mdot dq - dT/dq = C(q,dq) dq
G = simplify(gradient(U, q)); H = simplify(hessian(U, q));
Jc = simplify(jacobian(com, q));
% sanity: G_u = k qu - M g x_c, H_uu = k - M g z_c
assert(isAlways(simplify(G(4) - (k*qu - p.M*p.g*com(1))) == 0));
assert(isAlways(simplify(H(4,4) - (k - p.M*p.g*com(2))) == 0));
matlabFunction(Mm, 'File', fullfile(out, 'bal_M'), 'Vars', {q});
matlabFunction(cv, 'File', fullfile(out, 'bal_c'), 'Vars', {q, dq});
matlabFunction(G, 'File', fullfile(out, 'bal_G'), 'Vars', {q, k});
matlabFunction(H, 'File', fullfile(out, 'bal_H'), 'Vars', {q, k});
matlabFunction(com, 'File', fullfile(out, 'bal_com'), 'Vars', {q});
matlabFunction(Jc, 'File', fullfile(out, 'bal_Jcom'), 'Vars', {q});
matlabFunction([[0;0] hip sh head foot hand], 'File', fullfile(out, 'bal_points'), 'Vars', {q});
fprintf('generated into %s\n', out);
