function R = pcc_sim(o)
%PCC_SIM  Closed-loop simulation of the planar PCC soft robot (5 CC segments, 2 actuated
%   sections) driven by a CLIK law (Alg. 1 or Alg. 2 of the paper) and the low-level controller.
%
%   Block diagram (one fixed step, explicit Euler, step dt):
%     x_bar(t)  -> e = x_bar - x
%                  x = h(q)            (fb: measured robot configuration)
%                  x = h(q_bar)        (ff: CLIK state, no robot feedback)
%     CLIK      -> Alg. 1: qa_bar_dot = J' Gamma e,  qu_bar = phi(qa_bar)  (clik_alg1.m)
%                  Alg. 2: q_bar_dot from the projected field              (clik_alg2.m)
%                  the CLIK velocity is saturated to [-1, 1] (componentwise) and integrated
%     control   -> tau = G_a(q_bar) + Kp (qa_bar - qa)                      (low_level_controller.m)
%     robot     -> M q_ddot + C q_dot + G + D q_dot = A tau                (pcc_dynamics.m)
%   For Alg. 1 the reference qu_bar fed to the controller and to the ff error is the output of
%   the static branch at the PREVIOUS step (a one-step memory, as in the original Simulink
%   model); it is 0 at t = 0.
%
%   R = pcc_sim(o), o a struct with fields (defaults in brackets)
%     alg      1 | 2                                  [1]
%     mode     'ff' | 'fb'                            ['ff']
%     statics  Alg. 1 static branch: 'zero' | 'first' | 'second' (see clik_alg1.m)  ['zero']
%     kscale   stiffness scaling of the base robot (1: stiff, Fig. 18; 0.03: soft, Fig. 19)  [1]
%     targets  [2 x K] task targets                   [[0.6; 0.6]]
%     tswitch  [1 x K] time at which each target becomes active (tswitch(1) = 0)  [0]
%     tramp    if > 0, from the second target on the reference slides LINEARLY from the previous
%              target to the new one in tramp seconds, then stays there (sliding reference) [0]
%     T        final time [s]                         [5]
%     dt       integration step [s]                   [1e-5]
%     dtlog    logging period [s] (rounded to a multiple of dt)  [1e-3]
%   R struct with fields
%     t [N x 1], Q [N x n] robot curvatures, Qbar [N x n] CLIK output (reference [qa_bar; qu_bar]),
%     xbar [N x 2] task reference, tip [2 x N] robot tip position, L segment lengths,
%     escapes [1 x 2] number of steps with an Alg. 2 task / statics escape, o the options.
%
%   The robot starts at rest at q0 (pcc_params.m) and the CLIK state at the corresponding
%   actuation coordinates.
o = set_defaults(o, struct('alg', 1, 'mode', 'ff', 'statics', 'zero', 'kscale', 1, ...
    'targets', [0.6; 0.6], 'tswitch', 0, 'tramp', 0, 'T', 5, 'dt', 1e-5, 'dtlog', 1e-3));
mdl = pcc_model(o.kscale);
p = mdl.p;
n = p.n; m = p.m; L = mdl.L;

% (near-)singular solves in the model and in the escapes of Alg. 2 are expected: silence them
w0 = warning('off', 'MATLAB:nearlySingularMatrix');
w1 = warning('off', 'MATLAB:singularMatrix');
restore = onCleanup(@() warning([w0, w1]));

q = p.q0;
dq = zeros(n, 1);
q0_act = q_to_q_act(q);
if o.alg == 1
    qbar = q0_act(1:m);                  % CLIK state: actuated coordinates only
    qmem = zeros(n - m, 1);              % one-step memory of the static branch
else
    qbar = q0_act;                       % CLIK state: full configuration [qa; qu]
end

dt = o.dt;
N = round(o.T/dt);
nlog = round(o.dtlog/dt);
NL = floor(N/nlog) + 1;
R.t = zeros(NL, 1); R.Q = zeros(NL, n); R.Qbar = zeros(NL, n); R.xbar = zeros(NL, 2);
escapes = [0 0];
il = 0;
for k = 0:N
    t = k*dt;
    % task reference: active target, possibly sliding from the previous one
    kt = find(t >= o.tswitch, 1, 'last');
    xb = o.targets(:, kt);
    if kt > 1 && o.tramp > 0
        s = min(1, (t - o.tswitch(kt))/o.tramp);
        xb = (1 - s)*o.targets(:, kt - 1) + s*xb;
    end
    q_act = q_to_q_act(q);
    if o.alg == 1, qab = [qbar; qmem]; else, qab = qbar; end
    if mod(k, nlog) == 0
        il = il + 1;
        R.t(il) = t; R.Q(il, :) = q.'; R.Qbar(il, :) = qab.'; R.xbar(il, :) = xb.';
    end
    if k == N, break; end

    % task error
    if strcmp(o.mode, 'fb')
        x = forward_kinematics_planar_PCC(q, 1, L);
    else
        x = forward_kinematics_act(qab(1:m), qab(m+1:end), L);
    end
    xe = xb - x;

    % CLIK
    if o.alg == 1
        [dqa, qu] = clik_alg1(qbar, xe, o.statics, mdl);
        dqbar = min(max(dqa, -1), 1);
        qmem_next = qu;
    else
        [dq2, esc] = clik_alg2(qbar, xe, mdl);
        dqbar = min(max(dq2, -1), 1);
        if esc > 0
            if all(escapes == 0)                      % report the first escape only
                channel = {'task', 'statics'};
                warning('pcc_sim:alg2escape', ['Alg. 2 numerical escape used at t = %.4f s ' ...
                    '(%s channel); further escapes are only counted (R.escapes).'], t, channel{esc});
            end
            escapes(esc) = escapes(esc) + 1;
        end
    end

    % low-level controller and robot (explicit Euler)
    tau = low_level_controller(q_act, qab, mdl);
    ddq = pcc_dynamics(q, dq, tau, mdl);
    q = q + dt*dq;
    dq = dq + dt*ddq;
    qbar = qbar + dt*dqbar;
    if o.alg == 1, qmem = qmem_next; end
end
R.t = R.t(1:il); R.Q = R.Q(1:il, :); R.Qbar = R.Qbar(1:il, :); R.xbar = R.xbar(1:il, :);
R.tip = zeros(2, il);
for j = 1:il, R.tip(:, j) = forward_kinematics_planar_PCC(R.Q(j, :).', 1, L); end
R.L = L;
R.escapes = escapes;
R.o = o;
end

function o = set_defaults(o, d)
for f = fieldnames(d).'
    if ~isfield(o, f{1}), o.(f{1}) = d.(f{1}); end
end
end
