% RUN_ALL_DOUBLE_PENDULUM  Reproduce every double-pendulum figure of the paper
%   C. Della Santina, "Pushing the boundaries of actuators-to-task kinematic inversion:
%   from fully actuated to underactuated (soft) robots", IJRR.
%
%   System: tendon-driven double pendulum with parallel springs (Sec. 2.3), task = horizontal
%   end-effector position. Cases (see README.md for the full table):
%     Fig. 7      Algorithm 1 (Sec. 4.5): base actuation under gravity / general actuation, g = 0
%     Figs. 10-12 Algorithm 2 (Sec. 5.3): theta = pi/8, k = 0, xbar = 0.5 then -0.5
%     Fig. 13     Algorithm 2 closed-loop vector fields, 9 parameter sets
%     Figs. 15-16 Algorithm 2 as high-level feedback loop on the dynamic robot (Sec. 5.4)
%
%   Output: figures/*.png and *.eps, results/double_pendulum_results.mat (all trajectories).
%   Run from any folder; everything is written inside the double_pendulum folder.
%   Run time: about 1 minute (MATLAB R2025a, laptop).

root = fileparts(mfilename('fullpath'));
addpath(root); setup_paths();
figdir = fullfile(root, 'figures');
resdir = fullfile(root, 'results');
if ~exist(resdir, 'dir'), mkdir(resdir); end
t_clik = (0:0.01:20)';      % output grid of the CLIK integrations (Figs. 7, 10-12)
t_fb   = (0:0.01:90)';      % output grid of the dynamic simulations (Figs. 15-16)
tic_all = tic;

%% Fig. 7 - Algorithm 1, xbar = -1, Gamma = 1, alpha = pi/2
% (a,b) standard double pendulum actuated at the base: theta = 0, k1 = k2 = 0, g = 9.81
p = dp_params('theta', 0, 'k1', 0, 'k2', 0, 'g', 9.81, 'q0', [0; pi/2]);
R.alg1_base = sim_alg1(p, 'base_actuation', 1, @(t) -1, [0 20], t_clik);
save_figure(plot_pendulum_sequence(R.alg1_base.t, R.alg1_base.q, 12, 5.75, [-1.25 1.05], [-2.15 0.15], p), ...
            figdir, 'fig07a_alg1_base_actuation_robot');
save_figure(plot_q_time(R.alg1_base.t, R.alg1_base.q), figdir, 'fig07b_alg1_base_actuation_q');
save_figure(plot_task_time(R.alg1_base.t, R.alg1_base.x, R.alg1_base.xbar), figdir, 'fig07b_alg1_base_actuation_x');
% (c,d) general actuation without gravity: theta = pi/8, k1 = k2 = 10, g = 0.
% q0 = 0 (the initial actuated coordinate is qa(0) = A' q0 = 0)
p = dp_params('theta', pi/8, 'k1', 10, 'k2', 10, 'g', 0, 'q0', [0; 0]);
R.alg1_nogravity = sim_alg1(p, 'no_gravity', 1, @(t) -1, [0 20], t_clik);
save_figure(plot_pendulum_sequence(R.alg1_nogravity.t, R.alg1_nogravity.q, 12, 5.75, [-1.25 1.05], [-2.15 0.15], p), ...
            figdir, 'fig07c_alg1_no_gravity_robot');
save_figure(plot_q_time(R.alg1_nogravity.t, R.alg1_nogravity.q), figdir, 'fig07d_alg1_no_gravity_q');
save_figure(plot_task_time(R.alg1_nogravity.t, R.alg1_nogravity.x, R.alg1_nogravity.xbar), figdir, 'fig07d_alg1_no_gravity_x');
fprintf('Fig. 7 done (%.1f s)\n', toc(tic_all));

%% Figs. 10-12 - Algorithm 2 (CLIK only): theta = pi/8, k1 = k2 = 0, g = 9.81,
% initial guess q = (pi/2, pi/2), xbar = 0.5 on [0,10) and -0.5 on [10,20]
p = dp_params('theta', pi/8, 'k1', 0, 'k2', 0, 'g', 9.81, 'q0', [pi/2; pi/2]);
xbar_step = @(t) 0.5*(t < 10) - 0.5*(t >= 10);
gains  = [0 1; 1 0; 1 1];                                  % [GammaE GammaZ], legend order of Fig. 10
colors = [0.85 0.325 0.098; 0 0.447 0.741; 0.466 0.674 0.188];
RR = cell(1, 3); labels = cell(1, 3);
for k = 1:3
    RR{k} = sim_alg2(p, gains(k,2), gains(k,1), xbar_step, [0 10 20], t_clik);
    labels{k} = sprintf('$\\Gamma_{\\mathcal{E}} = %d$, $\\Gamma_{\\mathcal{Z}} = %d$', gains(k,1), gains(k,2));
end
R.alg2 = RR{3};                                            % GammaZ = GammaE = 1
R.alg2_only_task = RR{1}; R.alg2_only_statics = RR{2};
save_figure(plot_config_comparison(p, RR, labels, colors), figdir, 'fig10_alg2_configuration_comparison');
save_figure(plot_q_time(R.alg2.t, R.alg2.q), figdir, 'fig11a_alg2_q');
save_figure(plot_gu_time(R.alg2.t, R.alg2.gu), figdir, 'fig11b_alg2_gu');
save_figure(plot_task_time(R.alg2.t, R.alg2.x, R.alg2.xbar), figdir, 'fig11c_alg2_x');
i1 = R.alg2.t <= 10; i2 = R.alg2.t >= 10;
save_figure(plot_pendulum_sequence(R.alg2.t(i1), R.alg2.q(i1,:), 25, 5,  [-0.25 2], [-0.25 2], p), ...
            figdir, 'fig12a_alg2_robot_t0_10');
save_figure(plot_pendulum_sequence(R.alg2.t(i2), R.alg2.q(i2,:), 25, 15, [-1.1 0.9], [-0.15 2.1], p), ...
            figdir, 'fig12b_alg2_robot_t10_20');
fprintf('Figs. 10-12 done (%.1f s)\n', toc(tic_all));

%% Fig. 13 - Algorithm 2 vector fields (GammaZ = GammaE = 1, g = 9.81)
%            panel  theta  k1  k2  xbar
fields = {'a', 0,    10, 0,  0.5;
          'b', 0,    0,  0,  0.5;
          'c', 0,    0,  10, 0.5;
          'd', pi/8, 10, 0,  0.5;
          'e', pi/8, 0,  0,  0.5;
          'f', pi/8, 0,  10, 0.5;
          'g', pi/8, 10, 0,  -1;
          'h', pi/8, 0,  0,  -1;
          'i', pi/8, 0,  10, -1};
for k = 1:size(fields, 1)
    p = dp_params('theta', fields{k,2}, 'k1', fields{k,3}, 'k2', fields{k,4}, 'g', 9.81);
    name = sprintf('fig13%s_alg2_field_theta%s_k1_%d_k2_%d_xbar%s', fields{k,1}, ...
        strrep(sprintf('%.3f', fields{k,2}), '.', 'p'), fields{k,3}, fields{k,4}, ...
        strrep(strrep(sprintf('%g', fields{k,5}), '.', 'p'), '-', 'm'));
    save_figure(plot_vector_field(p, fields{k,5}), figdir, name);
end
fprintf('Fig. 13 done (%.1f s)\n', toc(tic_all));

%% Figs. 15-16 - Algorithm 2 as a high-level feedback loop on the dynamic robot
% theta = pi/8, k1 = 0, k2 = 10, g = 9.81; robot starts at rest in q0 = (0, pi/2), CLIK state
% at qbar = 0; GammaZ = 0.5, GammaE = 1; low-level tau = s*Ga(q) + 10 (qbar_a - qa) with
% s = 1 (nominal) and s = 0.5 (perturbed); xbar = -1 on [0,30), +1 on [30,60), -1 on [60,90]
p = dp_params('theta', pi/8, 'k1', 0, 'k2', 10, 'g', 9.81, 'q0', [0; pi/2]);
xbar_pulse = @(t) 1 - 2*(mod(t, 60) < 30);
variants  = {'FF', 'FB task', 'FB all'};
scenarios = {'nominal', 1; 'perturbed', 0.5};
panels    = {'abc', 'def'};
for s = 1:2
    RR = cell(1, 3);
    for v = 1:3
        RR{v} = sim_alg2_fb(p, 0.5, 1, variants{v}, 10, scenarios{s,2}, xbar_pulse, [0 30 60 90], t_fb);
    end
    R.(['fb_' scenarios{s,1}]) = cell2struct(RR, {'FF', 'FB_task', 'FB_all'}, 2);
    figs = plot_fb_comparison(RR, scenarios{s,1});
    what = {'q', 'qbar', 'x'};
    for k = 1:3
        save_figure(figs(k), figdir, sprintf('fig16%s_fb_%s_%s', panels{s}(k), scenarios{s,1}, what{k}));
    end
    if s == 1
        save_figure(plot_task_fb(RR{1}, RR{3}, p), figdir, 'fig15_fb_end_effector_paths');
    end
end
fprintf('Figs. 15-16 done (%.1f s)\n', toc(tic_all));

save(fullfile(resdir, 'double_pendulum_results.mat'), 'R');
fprintf('All double-pendulum figures written to %s (total %.1f s)\n', figdir, toc(tic_all));
