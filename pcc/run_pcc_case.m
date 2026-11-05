function R = run_pcc_case(name, varargin)
%RUN_PCC_CASE  Run one simulation case of the PCC soft-robot examples and save the result.
%
%   R = run_pcc_case(name)
%   R = run_pcc_case(name, 'T', T, 'outdir', dir)
%
%   name    case name, see src/pcc_cases.m (or call pcc_cases() for the list), e.g.
%           'stiff_a1ff' (Fig. 18), 'soft_a2fb' (Fig. 19), 'soft_a1fb_second',
%           'soft_tri_a2ff', ...
%   'T'     optional final time [s] overriding the one of the case (e.g. 0.5 for a quick test)
%   'outdir'  optional output folder (default: results/ next to this file)
%
%   R       result struct of pcc_sim (fields t, Q, Qbar, xbar, tip, L, escapes, o) plus
%           name and cpu (wall-clock time of the simulation [s]).
%   The result is saved as <outdir>/<name>.mat (variable R).
%
%   Run times (Apple-silicon laptop, one core): ~0.3-1 ms per integration step, i.e. a few
%   minutes for a stiff single-target case and 0.5-2 h for a soft case (2e6 steps).
root = setup_paths();
ip = inputParser;
ip.addParameter('T', []);
ip.addParameter('outdir', fullfile(root, 'results'));
ip.parse(varargin{:});

C = pcc_cases(name);
o = C.o;
if ~isempty(ip.Results.T), o.T = ip.Results.T; end

fprintf('%s: alg %d (%s, statics %s), kscale %g, T = %g s, dt = %g s ...\n', ...
    name, o.alg, o.mode, o.statics, o.kscale, o.T, o.dt);
t0 = tic;
R = pcc_sim(o);
R.cpu = toc(t0);
R.name = name;

if ~exist(ip.Results.outdir, 'dir'), mkdir(ip.Results.outdir); end
save(fullfile(ip.Results.outdir, [name '.mat']), 'R');

% summary: tip error at the end of each hold
fprintf('%s: done in %.0f s (%.2f ms/step)', name, R.cpu, 1e3*R.cpu/round(o.T/o.dt));
if o.alg == 2, fprintf(', Alg. 2 escapes (task, statics): %d, %d', R.escapes); end
fprintf('\n');
tend = [o.tswitch(2:end), o.T];
for k = 1:numel(tend)
    [~, i] = min(abs(R.t - min(tend(k), o.T)));
    fprintf('   t = %5.2f s: tip (%.4f, %.4f), target (%.2f, %.2f), error %.2f mm\n', ...
        R.t(i), R.tip(:, i), o.targets(:, k), 1e3*norm(R.tip(:, i) - o.targets(:, k)));
    if tend(k) >= o.T, break; end
end
end
