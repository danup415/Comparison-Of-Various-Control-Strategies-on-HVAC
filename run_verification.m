%% RUN_VERIFICATION  Extra experiments that test the paper's assumptions and explanations.
%   Needs the Parallel Computing Toolbox for speed (falls back to serial).
%   1) uncertainty analysis: 40 Latin-hypercube samples of all [ASSUMED] inputs x 5 controllers
%   2) explanation tests: heat_perfect, heat_backoff1, tariff_steep, hyb_noPI, hyb_nosafety
setup_paths();
jobs = {};
for sc = {'heat_perfect','heat_backoff1'}, for c = {'pid','mpc','hybrid'}, jobs(end+1,:) = {sc{1}, c{1}}; end, end %#ok<SAGROW>
for c = {'thermostat','pid','mpc','hybrid'}, jobs(end+1,:) = {'tariff_steep', c{1}}; end %#ok<SAGROW>
for c = {'thermostat','pid','mpc','rl','hybrid'}, jobs(end+1,:) = {'tariff_msedcl', c{1}}; end %#ok<SAGROW>
jobs(end+1,:) = {'hyb_noPI','hybrid'}; jobs(end+1,:) = {'hyb_nosafety','hybrid'};
for i = 1:40, for c = {'thermostat','pid','mpc','rl','hybrid'}, jobs(end+1,:) = {sprintf('uq%d',i), c{1}}; end, end %#ok<SAGROW>
scen = unique(jobs(:,1));
% free-floating references first (needed by every KPI), then the controller runs
if isempty(gcp('nocreate')), try, parpool; catch, end, end
p = gcp('nocreate');
if ~isempty(p)
    parfevalOnAll(p, @setup_paths, 0);
    parfor j = 1:numel(scen), run_case(scen{j}, 'off', 1, true); end
    parfor j = 1:size(jobs,1), run_case(jobs{j,1}, jobs{j,2}, 1, false); end
else
    for j = 1:numel(scen), run_case(scen{j}, 'off', 1, true); end
    for j = 1:size(jobs,1), run_case(jobs{j,1}, jobs{j,2}, 1, false); end
end
fprintf('verification runs complete\n');
