%RUN_ALL_PCC  Run every simulation case of the PCC soft-robot examples (src/pcc_cases.m) in
%   sequence and save the results in results/<case>.mat.
%
%   Run times (one core, Apple-silicon laptop, ~0.3-1 ms per step):
%     stiff single target (Fig. 18), 5e5 steps            ~3-10 min each
%     stiff triangle, 1.5e6 steps                         ~10-30 min each
%     soft single target (Fig. 19, 1st/2nd order), 2e6    ~15-40 min (Alg. 1), 1-2 h (Alg. 2)
%     soft triangle, dt = 3e-5, 2e6 steps                 ~15-55 min each
%   In total ~15-20 h sequentially. The cases are independent: to run them in parallel, either
%     - set USE_PARFOR = true below (Parallel Computing Toolbox), or
%     - start one MATLAB per case from a shell, e.g.
%         matlab -batch "run_pcc_case('soft_a2fb')" -singleCompThread &
%       (from this folder; one process per core).
%
%   SMOKE = true runs every case for 0.5 s only (results in results/smoke/), as a quick check
%   of the installation (~1-3 min in total).
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
