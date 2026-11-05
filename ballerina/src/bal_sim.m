function R = bal_sim(o)
%BAL_SIM  Fixed-step simulation of the elastic-pointe ballerina under a CLIK law + low-level PD.
%
%   R = bal_sim(o)    o = options struct (normally from bal_scenario); missing fields take the
%                     defaults below.
%
%   Plant (RK4, step dt):
%       M(q) q'' + c(q,q') + G(q; k_plant) = [tau; -b qu']
%   Low-level controller (the paper's, evaluated continuously in the plant state inside RK4):
%       tau = G_a(q_bar; k_model) + P (qa_bar - qa) - D qa'
%   CLIK (explicit Euler at the same dt; q_bar is held constant over each plant step):
%       v = bal_clik(...), saturated by bal_sat (+-1 rad/s, component-wise clipping by default)
%       Alg. 1: qa_bar <- qa_bar + dt v, qu_bar = phi(qa_bar) (Newton, warm start)
%       Alg. 2: q_bar  <- q_bar  + dt v
%   Error: ff  e = x_ref - h(q_bar);   fb  e = x_ref - h(q)   (h = CoM). In both variants J^+,
%   T and the statics term are evaluated at q_bar.
%
%   Option fields (defaults in brackets):
%     law       'a1ff' | 'a1fb' | 'a2ff' | 'a2fb'  (algorithm 1/2, feedforward/feedback)
%     targets   2xK CoM targets; hold [6] s per target; ramp [3] s linear slide to the next target
%               (the first target is a step); T total time [K*hold]
%     dt [1e-3] integration step; dtlog [1e-3] log interval
%     qa0       start joints; qu0 Newton warm start for phi(qa0); dqu_bar0 [0.15] initial offset of
%               qu_bar from the manifold (Alg. 2 only)
%     Gam, Gp   task gain Gamma and statics gain Gamma_perp
%     k_plant, k_model  pointe stiffness of plant and of CLIK model/low level (mismatch runs)
%     qu_stop [1.2] rad: the run stops (R.fell = true) if |qu| exceeds it
%     satmode ['clip'] | 'scale';  P, D, b  as in bal_params;  name  run name
%
%   Output R (one row per log sample): t, q, dq, q_bar, x (plant CoM), x_ref, x_bar = h(q_bar),
%   tau, Huu (plant, at q), Huu_bar, Gu_bar (model, at q_bar), sat (saturation active), svmin
%   (sigma_min of J at q_bar), err = |x_ref - x|, err_end_hold (1xK), targets, params, opts,
%   cpu_time, fell, law.

p = bal_params();
d = struct('law', 'a2ff', 'targets', [0;0.9], 'hold', 6, 'ramp', 3, 'T', [], 'dt', 1e-3, ...
    'dtlog', 1e-3, 'qa0', [0; 0.15; 0.2], 'qu0', 0, 'dqu_bar0', 0.15, 'Gam', 1.5, 'Gp', 3, ...
    'k_plant', p.k, 'k_model', p.k, 'qu_stop', 1.2, 'satmode', 'clip', 'name', '', 'P', p.P, 'D', p.D, 'b', p.b);
for f = fieldnames(d).', if ~isfield(o, f{1}), o.(f{1}) = d.(f{1}); end, end
K = size(o.targets, 2); if isempty(o.T), o.T = K*o.hold; end
alg = o.law(2) - '0'; fb = strcmp(o.law(3:4), 'fb');
dt = o.dt; N = round(o.T/dt); nlog = max(1, round(o.dtlog/dt));

% Initial state: plant at rest at the equilibrium of qa0 (plant stiffness); CLIK state at the
% model equilibrium of qa0 (Alg. 2: displaced by dqu_bar0 off the manifold).
qa0 = o.qa0(:); q = [qa0; bal_phi(qa0, o.k_plant, o.qu0)]; dq = zeros(4,1);
qm = [qa0; bal_phi(qa0, o.k_model, o.qu0)];
if alg == 1, qbar = qm; else, qbar = qm + [0;0;0; o.dqu_bar0]; end

% Log buffers
NL = floor(N/nlog) + 1; L.t = zeros(NL,1); L.q = zeros(NL,4); L.dq = zeros(NL,4); L.q_bar = zeros(NL,4);
L.x = zeros(NL,2); L.x_ref = zeros(NL,2); L.x_bar = zeros(NL,2); L.tau = zeros(NL,3);
L.Huu = zeros(NL,1); L.Huu_bar = zeros(NL,1); L.Gu_bar = zeros(NL,1); L.sat = false(NL,1); L.svmin = zeros(NL,1);
il = 0; fell = false; tc = tic;
for n = 0:N
    t = n*dt; xb = ref(t, o);
    % Alg. 1: project the CLIK state onto the equilibrium manifold, qu_bar = phi(qa_bar)
    if alg == 1, qbar(4) = bal_phi(qbar(1:3), o.k_model, qbar(4)); end
    x = bal_com(q); xbar = bal_com(qbar);
    if fb, e = xb - x; else, e = xb - xbar; end
    [v, info] = bal_clik(alg, qbar, e, o.k_model, o.Gam, o.Gp);
    vs = bal_sat(v, 1, o.satmode);
    % low-level torque law with q_bar frozen over the step (gravity compensation at q_bar)
    Gb = bal_G(qbar, o.k_model);
    ctl = @(q_, dq_) Gb(1:3) + o.P*(qbar(1:3) - q_(1:3)) - o.D*dq_(1:3);
    if mod(n, nlog) == 0
        il = il + 1; L.t(il) = t; L.q(il,:) = q.'; L.dq(il,:) = dq.'; L.q_bar(il,:) = qbar.';
        L.x(il,:) = x.'; L.x_ref(il,:) = xb.'; L.x_bar(il,:) = xbar.'; L.tau(il,:) = ctl(q, dq).';
        Hq = bal_H(q, o.k_plant); L.Huu(il) = Hq(4,4); L.Huu_bar(il) = info.Huu; L.Gu_bar(il) = Gb(4);
        L.sat(il) = any(abs(v) > 1); L.svmin(il) = info.sv(end);   % sigma_min of the reduced J at q_bar
    end
    if n == N, break; end
    if abs(q(4)) > o.qu_stop || any(~isfinite(q)), fell = true; break; end
    % plant: classical RK4 with q_bar frozen over the step
    f = @(s) plant(s, ctl, o);
    s = [q; dq]; k1 = f(s); k2 = f(s + dt/2*k1); k3 = f(s + dt/2*k2); k4 = f(s + dt*k3);
    s = s + dt/6*(k1 + 2*k2 + 2*k3 + k4); q = s(1:4); dq = s(5:8);
    % CLIK state: explicit Euler
    if alg == 1, qbar(1:3) = qbar(1:3) + dt*vs; else, qbar = qbar + dt*vs; end
end
cpu = toc(tc);

% Trim the logs and assemble the output
fn = fieldnames(L); for i = 1:numel(fn), L.(fn{i}) = L.(fn{i})(1:il, :); end
R = L; R.targets = o.targets; R.params = p; R.params.k_plant = o.k_plant; R.params.k_model = o.k_model;
R.params.P = o.P; R.params.D = o.D; R.params.b = o.b;
R.opts = o; R.cpu_time = cpu; R.fell = fell; R.law = o.law;
R.err = sqrt(sum((R.x - R.x_ref).^2, 2));
% CoM error at the end of each hold (NaN if the run stopped before)
R.err_end_hold = nan(1, K);
for j = 1:K
    i = find(R.t <= j*o.hold + 1e-9, 1, 'last');
    if R.t(i) >= j*o.hold - 1e-9, R.err_end_hold(j) = R.err(i); end
end
end

function ds = plant(s, ctl, o)
% State derivative of the plant, s = [q; dq]; the pointe damper acts on qu only.
q = s(1:4); dq = s(5:8);
tau = ctl(q, dq);
ddq = bal_M(q) \ ([tau; -o.b*dq(4)] - bal_c(q, dq) - bal_G(q, o.k_plant));
ds = [dq; ddq];
end

function xb = ref(t, o)
% CoM reference: hold o.hold s per target; first target a step at t = 0, then a linear slide
% over o.ramp s from the previous target at the start of each hold.
j = min(size(o.targets, 2), floor(t/o.hold + 1e-12) + 1); xb = o.targets(:, j);
if j > 1 && o.ramp > 0
    s = min(1, (t - (j-1)*o.hold)/o.ramp); xb = (1 - s)*o.targets(:, j-1) + s*xb;
end
end
