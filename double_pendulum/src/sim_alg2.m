function R = sim_alg2(p, GammaZ, GammaE, xbar_fun, breaks, t_out, opts)
%SIM_ALG2  Integrate the Algorithm 2 CLIK dynamics, Eq. (42) (Sec. 5.3, Figs. 10-12).
%   Plain-MATLAB port of the Simulink scheme IK_a2_scheme.slx: integrator on (qa,qu) with the
%   CLIK law of CLIK_ALG2. Not a robot simulation (only the final configuration is reachable).
%
%   p      : parameters (dp_params); p.q0 is the initial guess (joint angles)
%   GammaZ, GammaE : gains of the task / statics terms
%   xbar_fun, breaks, t_out, opts : as in SIM_ALG1
%   R      : struct with t, q_act (N x 2), q (N x 2), x, xbar, gu

if nargin < 7, opts = default_ode_options(); end
q_act0 = q_to_q_act(p.q0(1), p.q0(2), p.alpha, p.r1, p.r2);
rhs = @(t, qact) clik_alg2(qact, xbar_fun(t), p, GammaZ, GammaE);
t_out = t_out(:);
R.t = t_out;
R.q_act = ode_piecewise(rhs, breaks, q_act0, t_out, opts);
R.q = zeros(numel(R.t), 2); R.x = zeros(size(R.t)); R.gu = R.x;
for i = 1:numel(R.t)
    T = dp_act_terms(R.q_act(i,1), R.q_act(i,2), p);
    R.q(i, :) = T.q.'; R.x(i) = T.x; R.gu(i) = T.Gu;
end
R.xbar = arrayfun(xbar_fun, R.t);
end
