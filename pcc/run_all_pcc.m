%RUN_ALL_PCC  Run every simulation case of the PCC soft-robot examples (src/pcc_cases.m) in
%   sequence and save the results in results/<case>.mat. Afterwards run make_figures_pcc and
%   make_videos_pcc.
%
%   Run times (one core each, Apple-silicon laptop; see README.md for the table):
%     stiff single target (Fig. 18), 5e5 steps            ~3 min (Alg. 1), ~11 min (Alg. 2)
%     stiff triangle, 1.5e6 steps                         ~10 min (Alg. 1), ~1 h (Alg. 2)
%     soft single target / triangle, 2e6 steps            ~15 min (Alg. 1, zero/1st order),
%                                                         ~1.3 h (Alg. 1 2nd order, Alg. 2)
%   In total ~16 h sequentially. The cases are independent: to run them in parallel, either
%     - set USE_PARFOR = true below (Parallel Computing Toolbox), or
%     - start one MATLAB per case from a shell, e.g.
%         matlab -batch "run_pcc_case('soft_a2fb')" -singleCompThread &
%       (from this folder; one process per core).
%
%   SMOKE = true runs every case for 0.5 s only (results in results/smoke/), as a quick check
%   of the installation (~15-20 min in total).
SMOKE = false;
USE_PARFOR = false;

root = setup_paths();
names = {pcc_cases().name};
args = {};
if SMOKE, args = {'T', 0.5, 'outdir', fullfile(root, 'results', 'smoke')}; end

if USE_PARFOR
    parfor i = 1:numel(names)
        run_pcc_case(names{i}, args{:});
    end
else
    for i = 1:numel(names)
        run_pcc_case(names{i}, args{:});
    end
end
