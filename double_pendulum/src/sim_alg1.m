function R = sim_alg1(p, example, Gamma, xbar_fun, breaks, t_out, opts)
%SIM_ALG1  Integrate the Algorithm 1 CLIK dynamics (Sec. 4.5, Fig. 7).
%   Plain-MATLAB port of the Simulink scheme IK_a1_scheme.slx: integrator on qa with the CLIK
%   law of CLIK_ALG1; qu, q, x and Gu are outputs. This is not a robot simulation: every
%   configuration is an attainable equilibrium by construction.
%
%   p        : parameters (dp_params), p.q0 gives the initial qa = A' q0 (as in the .slx)
%   example  : equilibrium parametrization ('base_actuation' or 'no_gravity')
%   Gamma    : CLIK gain        xbar_fun : @(t) task reference
%   breaks   : [t0 ... tf] restart instants (discontinuities of xbar)
%   t_out    : output times     opts : odeset options (default DEFAULT_ODE_OPTIONS)
%   R        : struct with t, qa, qu, q (N x 2), x, xbar, gu

if nargin < 7, opts = default_ode_options(); end
q_act0 = q_to_q_act(p.q0(1), p.q0(2), p.alpha, p.r1, p.r2);
rhs = @(t, qa) clik_alg1(qa, xbar_fun(t), p, example, Gamma);
t_out = t_out(:);
R.t  = t_out;
R.qa = ode_piecewise(rhs, breaks, q_act0(1), t_out, opts);
R.qu = zeros(size(R.t)); R.q = zeros(numel(R.t), 2); R.x = R.qu; R.gu = R.qu;
for i = 1:numel(R.t)
    R.qu(i) = alg1_parametrization(R.qa(i), p, example);
    T = dp_act_terms(R.qa(i), R.qu(i), p);
    R.q(i, :) = T.q.'; R.x(i) = T.x; R.gu(i) = T.Gu;
end
R.xbar = arrayfun(xbar_fun, R.t);
end
