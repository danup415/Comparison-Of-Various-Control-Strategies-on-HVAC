setup_paths; cfg = scenario_config('base'); cfg.sensor_noise_std = 0; cfg.ctrl.mpc.fc_sigma = 0; cfg.ctrl.mpc.sol_err = 0;
ctx = setup_study(cfg); P = load('results/policies/dqn_base_s1.mat');
kr = [96*125+1, 96*139];
for c = {'thermostat','pid','mpc','rl','hybrid'}
  o = struct('k_range', kr); if strcmp(c{1},'rl'), o.rl_policy = P.pol; end
  R = simulate(ctx, cfg, c{1}, 1, o);
  fprintf('%-10s sumTa=%.10f  sumP=%.10f\n', c{1}, sum(R.Ta(:)), sum(R.P));
end
