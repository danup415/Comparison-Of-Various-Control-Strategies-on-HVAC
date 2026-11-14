function varargout = sl_engine(op, varargin)
%SL_ENGINE  Bridge between the Simulink wrapper model and the MATLAB simulation engine.
%   sl_engine('init', scenario, ctrl, seed)   build context + controller (persistent)
%   [Ta, P, u] = sl_engine('step', k)         advance one 15-min step (called by Simulink)
%   The Simulink model therefore executes exactly the same code as simulate.m;
%   check_simulink_equivalence.m verifies this numerically.
persistent E
switch op
    case 'init'
        [scenario, ctrl, seed] = varargin{:};
        cfg = scenario_config(scenario); ctx = setup_study(cfg); opts = struct();
        if strcmp(ctrl, 'rl')
            P = load(fullfile(cfg.out_dir, 'policies', sprintf('dqn_base_s%d.mat', seed)));
            opts.rl_policy = P.pol;
        end
        E = sim_init(ctx, cfg, ctrl, seed, opts);
        varargout = {};
    case 'step'
        k = double(varargin{1});
        [E, u, lg] = sim_step(E, k);
        varargout = {E.s.x(1:3), lg.P, u(:)};
    otherwise
        error('sl_engine:op', 'Unknown op %s', op);
end
end
