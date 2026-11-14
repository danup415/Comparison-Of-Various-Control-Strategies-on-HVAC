function ok = check_simulink_equivalence(ctrl, n_days)
%CHECK_SIMULINK_EQUIVALENCE  Run n_days in MATLAB (simulate.m) and in the Simulink
%   wrapper, and assert that zone temperatures and power agree to 1e-9.
%   Requires MATLAB + Simulink. Run from the project root after setup_paths.
if nargin < 1, ctrl = 'pid'; end
if nargin < 2, n_days = 7; end
cfg = scenario_config('base'); ctx = setup_study(cfg); opts = struct('k_range', [1 n_days*96]);
if strcmp(ctrl, 'rl')
    P = load(fullfile(cfg.out_dir, 'policies', 'dqn_base_s1.mat')); opts.rl_policy = P.pol;
end
R = simulate(ctx, cfg, ctrl, 1, opts);
sl_engine('init', 'base', ctrl, 1);
mdl = build_simulink_wrapper(n_days);
out = sim(mdl);
Ta_sl = squeeze(out.get('sl_Ta')); if size(Ta_sl,1) == 3, Ta_sl = Ta_sl'; end
P_sl  = squeeze(out.get('sl_P'));
eT = max(abs(Ta_sl(:) - reshape(R.Ta, [], 1)));
eP = max(abs(P_sl(:) - R.P(:)));
fprintf('max |dTa| = %.3g K, max |dP| = %.3g kW\n', eT, eP);
ok = eT < 1e-9 && eP < 1e-9;
if ~ok, warning('Simulink wrapper and MATLAB engine differ'); end
end
