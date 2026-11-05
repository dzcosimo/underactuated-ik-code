%RUN_ALL_BALLERINA  Entry point: run every released ballerina simulation and print the summary.
%
%   Run from MATLAB (any working directory):   run_all_ballerina
%   or from a shell:  matlab -batch "run('path/to/ballerina/run_all_ballerina.m')"
%
%   1. adds src/ and model/ to the path (regenerates model/ with gen_symbolic if it is missing;
%      this needs the Symbolic Math Toolbox, the shipped model/ does not);
%   2. runs the 7 scenarios of bal_scenario and saves results/<name>.mat (about 0.5 s each);
%   3. prints the results tables (summarize_results).

root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'src'), fullfile(root, 'model'));
if ~exist(fullfile(root, 'model', 'bal_M.m'), 'file'), run(fullfile(root, 'model', 'gen_symbolic.m')); end
names = {'main_a1ff', 'main_a1fb', 'main_a2ff', 'main_a2fb', 'mismatch_a2ff', 'mismatch_a2fb', 'soft_a2ff'};
for i = 1:numel(names), bal_run_case(names{i}); end
addpath(root); summarize_results();
