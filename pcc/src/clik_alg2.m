function [q_dot, escape] = clik_alg2(q_act, x_err, mdl)
%CLIK_ALG2  Algorithm 2 of the paper: constrained CLIK on the full configuration in actuation
%   coordinates [qa; qu], with no explicit solution of the statics. The vector field is the sum
%   of a task component, tangent to the static manifold G_u = 0, and a statics-relaxation
%   component that drives G_u to zero without moving the task:
%     q_dot = P_E J_a^-1 Gamma_par (x_bar - x)  -  P_Z H_uu^-1 Gamma_perp G_u
%   with the projectors P_E, P_Z of Eq. (42) (projector_PE.m, projector_PZ.m),
%   Gamma_par = 3, Gamma_perp = 10.
%
%   Inputs
%     q_act  [n x 1] CLIK state [qa_bar; qu_bar] (n > m)
%     x_err  [m x 1] task error x_bar - x
%     mdl    model struct (pcc_model.m)
%   Outputs
%     q_dot  [n x 1] CLIK velocity (saturated by the caller)
%     escape 0 = regular step; 1 = task-channel escape; 2 = statics-channel escape (see below)
%
%   Numerical safeguards of the original implementation (kept unchanged):
%   - the state is evaluated at the floored curvature (floor_curvature.m);
%   - task-channel escape: if J_a is (near) singular (rcond < 1e-10 or ||J_a||_F < 1e-10) or
%     ||P_E||_F > 1e3, the step is replaced by the bounded vector 10 sin(sin(q_act)).^3 (so that
%     the simulation does not stall) and the statics channel is skipped;
%   - statics-channel escape: if H_uu is (near) singular or ||P_Z||_F > 1e3, the statics
%     component is replaced by 1e-3 (1, ..., 1).
%   The escapes are reported through the output 'escape' (counted by pcc_sim) instead of being
%   printed at every step.
Gamma_par  = 3;
Gamma_perp = 10;
n = numel(q_act);
m = numel(x_err);
assert(n > m, 'clik_alg2: requires n > m.');
escape = 0;

% evaluate the model at the floored curvature
q_act = q_to_q_act(floor_curvature(q_act_to_q(q_act(1:m), q_act(m+1:end))));
qa = q_act(1:m);
qu = q_act(m+1:end);

Ja  = JaFunction(qa, qu);                  % [m x m]      dh/dqa
Huu = Huu_total(qa, qu, mdl);              % [(n-m) x (n-m)]
Gu  = Gu_total(qa, qu, mdl);               % [(n-m) x 1]  statics residual

% ---- task component: P_E J_a^-1 Gamma_par x_err
P_E = projector_PE([qa; qu], mdl);         % [n x m]
if (rcond(Ja) < 1e-10) || (norm(Ja, 'fro') < 1e-10) || (norm(P_E, 'fro') > 1e3)
    q_dot = 10*sin(sin(q_act)).^3;         % task-channel escape
    escape = 1;
    return
end
task_component = P_E*(Ja \ (Gamma_par*x_err));

% ---- statics relaxation: - P_Z H_uu^-1 Gamma_perp G_u
P_Z = projector_PZ([qa; qu], mdl);         % [n x (n-m)]
if (rcond(Huu) < 1e-10) || (norm(Huu, 'fro') < 1e-10) || (norm(P_Z, 'fro') > 1e3)
    statics_component = 1e-3*ones(size(q_act));   % statics-channel escape
    escape = 2;
else
    statics_component = -P_Z*(Huu \ (Gamma_perp*Gu));
end

q_dot = task_component + statics_component;
end
