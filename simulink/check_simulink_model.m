function T = check_simulink_model(ctrls, n_days)
%CHECK_SIMULINK_MODEL  Run the block-diagram Simulink model and simulate.m side by side
%   and report the largest difference in every logged signal.
%   T = check_simulink_model({'thermostat','pid','mpc','rl','hybrid'}, 7)
%   Requires MATLAB + Simulink. Run from the project root after setup_paths.
if nargin < 1, ctrls = {'thermostat', 'pid', 'mpc', 'rl', 'hybrid'}; end
if nargin < 2, n_days = 7; end
cfg = scenario_config('base'); ctx = setup_study(cfg);
n = n_days*96; rows = {};
for i = 1:numel(ctrls)
    c = ctrls{i}; opts = struct('k_range', [1 n]);
    if strcmp(c, 'rl')
        P = load(fullfile(cfg.out_dir, 'policies', 'dqn_base_s1.mat')); opts.rl_policy = P.pol;
    end
    R = simulate(ctx, cfg, c, 1, opts);
    SL = sl_blocks('init', 'base', c, 1, n_days); assignin('base', 'SL', SL);
    mdl = build_pune_simulink_model(n_days);
    t0 = tic; out = sim(mdl); t_sl = toc(t0);
    g = @(nm) squeeze(out.get(nm));
    Ta = g('sl_Ta'); Tm = g('sl_Tm'); Pw = g('sl_P'); u = g('sl_u');
    W = g('sl_W'); CO2 = g('sl_CO2'); PM = g('sl_PM');
    if size(Ta,1) == 3, Ta = Ta'; Tm = Tm'; u = u'; end
    % the State-Space block outputs x(k); simulate.m logs x(k+1) after each step
    dTa = max(abs(Ta(2:n,:) - R.Ta(1:n-1,:)), [], 'all');
    dTm = max(abs(Tm(2:n,:) - R.Tm(1:n-1,:)), [], 'all');
    du  = max(abs(u - R.Qcmd), [], 'all');
    dP  = max(abs(Pw(:) - R.P(:)));
    dW  = max(abs(W(:) - R.W(:)));
    dC  = max(abs(CO2(:) - R.CO2(:)));
    dPM = max(abs(PM(:) - R.PM(:)));
    rows(end+1, :) = {c, dTa, dTm, du, dP, dW, dC, dPM, t_sl}; %#ok<AGROW>
    fprintf('%-10s |dTa| %.2e K  |dTm| %.2e K  |du| %.2e kW  |dP| %.2e kW  |dW| %.2e  |dCO2| %.2e ppm  |dPM| %.2e  (%.0f s)\n', ...
        c, dTa, dTm, du, dP, dW, dC, dPM, t_sl);
end
T = cell2table(rows, 'VariableNames', {'controller', 'max_dTa_K', 'max_dTm_K', 'max_du_kW', ...
    'max_dP_kW', 'max_dW', 'max_dCO2_ppm', 'max_dPM', 'sim_time_s'});
if ~exist(fullfile(cfg.out_dir, 'tables'), 'dir'), mkdir(fullfile(cfg.out_dir, 'tables')); end
writetable(T, fullfile(cfg.out_dir, 'tables', 'simulink_check.csv'));
end
