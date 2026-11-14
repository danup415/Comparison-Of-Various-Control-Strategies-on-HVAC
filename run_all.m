%% RUN_ALL  Reproduce every number, table and figure in the manuscript.
%   MATLAB R2019b+ (no toolboxes) or GNU Octave 8+.  From the project root:
%       >> run_all
%   Runs are cached in results/runs/*.mat; delete that folder to recompute.
%   Approximate cost on one core: thermostat/PID/DQN ~20 s per simulated year,
%   E-MPC/Hybrid ~6 min per year, DQN training ~8 min per seed.
%   Parallel option: start several MATLAB/Octave sessions with different PART values.
setup_paths();
if ~exist('PART', 'var'), PART = 'all'; end
cfg = default_config();
C = cfg.controllers;

jobs = {};
% 1) main study: 5 controllers x 5 seeds (seed 1 keeps full time series for figures)
for s = cfg.seeds, for i = 1:numel(C), jobs(end+1,:) = {'base', C{i}, s, s == 1}; end, end %#ok<SAGROW>
% 2) robustness / sensitivity (seed 1, all controllers; DQN re-uses its base policy)
for scn = {'heatwave','mass_low','mass_high','flat_tariff','no_mismatch'}
    for i = 1:numel(C), jobs(end+1,:) = {scn{1}, C{i}, 1, false}; end %#ok<SAGROW>
end
% 3) MPC ablations
for scn = {'mpc_perfect','horizon6','horizon12'}
    for c = {'mpc','hybrid'}, jobs(end+1,:) = {scn{1}, c{1}, 1, false}; end %#ok<SAGROW>
end
% 4) DQN action-resolution ablation
for scn = {'rl_levels3','rl_levels9','rl_noshield'}, jobs(end+1,:) = {scn{1}, 'rl', 1, false}; end %#ok<SAGROW>
% 5) ventilation / filtration architecture (IAQ)
for scn = {'iaq_nofilter','vent_humidity','vent_off'}
    for c = {'thermostat','hybrid'}, jobs(end+1,:) = {scn{1}, c{1}, 1, true}; end %#ok<SAGROW>
end

switch PART                     % simple manual parallelism across sessions
    case 'all',  sel = 1:size(jobs,1);
    case 'odd',  sel = 1:2:size(jobs,1);
    case 'even', sel = 2:2:size(jobs,1);
    case 'post', sel = [];
end
fprintf('Running %d of %d jobs\n', numel(sel), size(jobs,1));
for j = sel
    run_case(jobs{j,1}, jobs{j,2}, jobs{j,3}, jobs{j,4});
end

if any(strcmp(PART, {'all','post'}))
    A = aggregate_results(cfg);
    make_figures(cfg);
    write_summary(cfg, A);
    fprintf('\nDone. Tables in %s/tables, figures in %s/figures, summary in %s/summary.md\n', ...
            cfg.out_dir, cfg.out_dir, cfg.out_dir);
end
