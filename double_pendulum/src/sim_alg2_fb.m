function R = sim_alg2_fb(p, GammaZ, GammaE, variant, Kp, Ga_scale, xbar_fun, breaks, t_out, opts)
%SIM_ALG2_FB  Dynamic simulation of the hierarchical architecture of Fig. 14 (Sec. 5.4,
%   Figs. 15-16): Algorithm 2 CLIK (CLIK_ALG2_FB) -> low-level controller
%   (LOW_LEVEL_CONTROLLER) -> double pendulum dynamics (DP_DYNAMICS).
%   Plain-MATLAB port of the Simulink scheme IK_a2_fb_scheme.slx.
%
%   State: y = [q (2); qdot (2); qbar_act (2)]. Initial conditions as in the .slx:
%   q(0) = p.q0, qdot(0) = p.qd0, qbar_act(0) = 0.
%   variant : 'FF' | 'FB task' | 'FB all'      Kp, Ga_scale : low-level controller
%   other inputs as in SIM_ALG1.
%   R : struct with t, q (N x 2, robot), qbar (N x 2, CLIK state in joint angles),
%       qbar_act, x (robot task), xbar.
%
%   The .slx wrapped the joint angles of the robot into [-pi, pi]; the trajectories of the
%   paper never leave that interval, which is checked below instead of wrapping.

if nargin < 10, opts = default_ode_options(); end
y0 = [p.q0; p.qd0; 0; 0];
rhs = @(t, y) fb_rhs(t, y, p, GammaZ, GammaE, variant, Kp, Ga_scale, xbar_fun);
t_out = t_out(:);
Y = ode_piecewise(rhs, breaks, y0, t_out, opts);
R.t = t_out;
R.q = Y(:, 1:2);
R.qbar_act = Y(:, 5:6);
R.qbar = zeros(numel(R.t), 2); R.x = zeros(size(R.t));
for i = 1:numel(R.t)
    R.qbar(i, :) = q_act_to_q(R.qbar_act(i,1), R.qbar_act(i,2), p.alpha, p.r1, p.r2).';
    R.x(i) = dp_task(R.q(i, :).', p);
end
R.xbar = arrayfun(xbar_fun, R.t);
if any(abs(R.q(:)) > pi)
    warning('sim_alg2_fb: |q| > pi; the original Simulink model wrapped the angles here.');
end
end

function dy = fb_rhs(t, y, p, GammaZ, GammaE, variant, Kp, Ga_scale, xbar_fun)
q = y(1:2); qd = y(3:4); qbar_act = y(5:6);
q_act_m = q_to_q_act(q(1), q(2), p.alpha, p.r1, p.r2);   % measured configuration
x_m = dp_task(q, p);                                        % measured task
qbar_act_dot = clik_alg2_fb(qbar_act, xbar_fun(t), q_act_m, x_m, p, GammaZ, GammaE, variant);
tau = low_level_controller(q_act_m, qbar_act, p, Kp, Ga_scale);
dy = [qd; dp_dynamics(q, qd, tau, p); qbar_act_dot];
end
