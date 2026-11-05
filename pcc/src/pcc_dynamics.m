function q_ddot = pcc_dynamics(q, q_dot, tau, mdl)
%PCC_DYNAMICS  Forward dynamics of the planar PCC robot:
%     M(q) q_ddot + C(q, q_dot) q_dot + G(q) + D q_dot = A(q) tau
%   q, q_dot [n x 1] curvatures and their rates (body coordinates), tau [m x 1] actuation
%   (one generalized force per actuated section), mdl model struct (pcc_model.m).
%   q_ddot [n x 1]. M, C, D, A are the generated functions of model/generated/; G includes the
%   stiffness scaling of mdl (G_total.m).
%   The model terms are evaluated at the floored curvature (floor_curvature.m); q_dot is not
%   modified. M^-1 is formed explicitly as M \ I, as in the original simulator.
q = floor_curvature(q(:));
q_dot = q_dot(:);
tau = tau(:);
M = M_fcn(q);
M_inv = M \ eye(size(M));
C_term = C_fcn(q, q_dot)*q_dot;
G_term = G_total(q, mdl);
D_term = D_fcn(q)*q_dot;
f = A_fcn(q)*tau;
q_ddot = M_inv*(-C_term - G_term - D_term + f);
end
