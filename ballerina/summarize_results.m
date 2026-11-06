function S = summarize_results()
%SUMMARIZE_RESULTS  Results tables of the saved ballerina runs (results/*.mat), printed as Markdown.
%
%   S = summarize_results()   prints two tables (the same as in README.md) and the CPU times,
%                             and returns the computed values per run (struct array).
%
%   Table 1, per run: task gain Gamma; CoM error |x_ref - x| at the end of each hold and at the
%     end of the run; per hold, the time after which the error stays below 5 mm until the end
%     of that hold ("--": not below 5 mm at the end of the hold; the hold start: below 5 mm all
%     along); min H_uu = k - M g z_c of the plant; min Schur margin of the frozen closed-loop
%     Hessian (bal_margin); min k/(M g z_c); max |qu|; min sigma_min(J) at q_bar; fraction of
%     samples with active saturation.
%   Table 2, at the end of each hold: |x_ref - h(q_bar)| (error of the CLIK reference) and
%     |h(q_bar) - x| (plant vs model); min height of the working foot and of every other drawn
%     point (hip, shoulder, head, hand); arm elevation (hand above shoulder) [deg].
%   Margins and heights are evaluated every 10 ms; everything else on every 1 ms sample.

root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'src'), fullfile(root, 'model'));
names = {'main_a1ff', 'main_a1fb', 'main_a2ff', 'main_a2fb', 'mismatch_a2ff', 'mismatch_a2fb', 'soft_a2ff'};
S = struct([]);
for i = 1:numel(names)
    f = fullfile(root, 'results', [names{i} '.mat']); if ~exist(f, 'file'), continue; end
    L = load(f, 'R'); R = L.R; p = R.params; h = R.opts.hold; K = size(R.targets, 2);
    % margins and heights along the run (every 10 ms)
    step = max(1, round(0.01/(R.t(2) - R.t(1)))); idx = 1:step:numel(R.t);
    sch = inf; ratio = inf; zf = inf; zo = inf;
    for j = idx
        [~, s] = bal_margin(R.q(j,:).', p.k_plant, p.P); sch = min(sch, s);
        ratio = min(ratio, p.k_plant/(p.M*p.g*R.x(j,2)));
        P = bal_points(R.q(j,:).'); zf = min(zf, P(2,5)); zo = min(zo, min(P(2, [2 3 4 6])));
    end
    % per hold: settling time below 5 mm, arm elevation, error split at the end of the hold
    t5 = nan(1, K); el = nan(1, K); eref = nan(1, K); epl = nan(1, K);
    for k = 1:K
        in = find(R.t >= (k-1)*h - 1e-9 & R.t <= k*h + 1e-9); if isempty(in) || R.t(in(end)) < k*h - 1e-9, continue; end
        last = find(R.err(in) >= 5e-3, 1, 'last');
        if isempty(last), t5(k) = R.t(in(1)); elseif last < numel(in), t5(k) = R.t(in(last + 1)); end
        ie = in(end); P = bal_points(R.q(ie,:).'); d = P(:,6) - P(:,3); el(k) = atan2d(d(2), abs(d(1)));
        eref(k) = norm(R.x_ref(ie,:) - R.x_bar(ie,:)); epl(k) = norm(R.x_bar(ie,:) - R.x(ie,:));
    end
    S(end+1).name = names{i}; S(end).R = R; S(end).sch = sch; S(end).ratio = ratio; S(end).zf = zf; S(end).zo = zo; %#ok<AGROW>
    S(end).t5 = t5; S(end).el = el; S(end).eref = eref; S(end).epl = epl;
end
if isempty(S), fprintf('No results in %s: run run_all_ballerina first.\n', fullfile(root, 'results')); return; end

j3 = @(v, fmt) strjoin(arrayfun(@(u) num(u, fmt), v, 'UniformOutput', false), ' / ');
fprintf('\nTable 1\n\n');
fprintf('| run | Gamma | CoM err at end of hold 1 / 2 / 3 [mm] | final err [mm] | below 5 mm from [s], per hold | min H_uu | min Schur | min k/(M g z_c) | max abs(qu) [rad] | min sigma(J) | sat. frac. |\n');
fprintf('|---|---|---|---|---|---|---|---|---|---|---|\n');
for s = S
    R = s.R; note = ''; if R.fell, note = sprintf(' (stopped: abs(qu) > %.1f at t = %.2f s)', R.opts.qu_stop, R.t(end)); end
    fprintf('| %s | %.1f | %s | %.2f | %s | %.1f | %.1f | %.2f | %.3f | %.3f | %.3f |\n', [s.name note], R.opts.Gam, ...
        j3(1e3*R.err_end_hold, '%.2f'), 1e3*R.err(end), j3(s.t5, '%.2f'), min(R.Huu), s.sch, s.ratio, ...
        max(abs(R.q(:,4))), min(R.svmin), mean(R.sat));
end
fprintf('\nTable 2\n\n');
fprintf('| run | abs(x_ref - h(q_bar)) at hold ends [mm] | abs(h(q_bar) - x) at hold ends [mm] | min z foot [m] | min z other [m] | arm elevation at hold ends [deg] |\n');
fprintf('|---|---|---|---|---|---|\n');
for s = S
    fprintf('| %s | %s | %s | %.3f | %.3f | %s |\n', s.name, j3(1e3*s.eref, '%.2f'), j3(1e3*s.epl, '%.2f'), ...
        s.zf, s.zo, j3(s.el, '%.0f'));
end
fprintf('\nCPU time of the simulation loop [s] (machine dependent): %s\n', ...
    strjoin(arrayfun(@(s) sprintf('%s %.2f', s.name, s.R.cpu_time), S, 'UniformOutput', false), ', '));
end

function c = num(u, fmt)
% number -> string, NaN -> "--"
if isnan(u), c = '--'; else, c = sprintf(fmt, u); end
end
