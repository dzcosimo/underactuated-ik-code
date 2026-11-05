function R = bal_run_case(name, varargin)
%BAL_RUN_CASE  Run one named scenario and save results/<name>.mat.
%
%   R = bal_run_case(name)                      e.g. bal_run_case('main_a2fb')
%   R = bal_run_case(name, field, value, ...)   override fields of bal_scenario(name), e.g.
%       bal_run_case('main_a2ff', 'Gam', 1.0, 'outname', 'main_a2ff_gam1')
%       'outname' sets the file name (default: name); 'satmode', 'scale' switches saturation.
%
%   Saved variables: t, q, q_bar, x (plant CoM), x_ref, targets, params, cpu_time and R (the
%   full output of bal_sim, see there). Prints a one-line summary.

root = fileparts(fileparts(mfilename('fullpath')));       % ballerina/
o = bal_scenario(name); for i = 1:2:numel(varargin), o.(varargin{i}) = varargin{i+1}; end
% J J' is solved with mrdivide; near low sigma_min(J) MATLAB would print a warning at every step.
% It is silenced only during this run (state restored afterwards); sigma_min(J) is logged in
% R.svmin and reported by summarize_results instead.
ws = warning('off', 'MATLAB:nearlySingularMatrix'); c = onCleanup(@() warning(ws));
R = bal_sim(o);
t = R.t; q = R.q; q_bar = R.q_bar; x = R.x; x_ref = R.x_ref; targets = R.targets; params = R.params; %#ok<NASGU>
cpu_time = R.cpu_time; %#ok<NASGU>
rdir = fullfile(root, 'results'); if ~exist(rdir, 'dir'), mkdir(rdir); end
fname = o.name; if isfield(o, 'outname'), fname = o.outname; end
save(fullfile(rdir, [fname '.mat']), 't', 'q', 'q_bar', 'x', 'x_ref', 'targets', 'params', 'cpu_time', 'R');
fprintf('%s: err at end of holds [%s] mm, max|qu| %.3f, min Huu(plant) %.1f, min sigma(J) %.4f, sat %.3f, fell %d, cpu %.2f s\n', fname, ...
    num2str(1e3*R.err_end_hold, '%.2f '), max(abs(R.q(:,4))), min(R.Huu), min(R.svmin), mean(R.sat), R.fell, R.cpu_time);
end
