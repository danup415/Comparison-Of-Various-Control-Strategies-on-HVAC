function [u, c] = ctrl_rl(c, obs, mode)
%CTRL_RL  Greedy deployment of a trained parameter-shared double DQN (one agent per zone).
if nargin > 2 && strcmp(mode, 'init')
    c.levels = c.policy.levels;
    c.umax = c.cfg.vrf.Qiu_max * c.cfg.vrf.SHR;
    c.af = zeros(1,3);
    u = []; return;
end
p = c.cfg.ctrl.mpc;
fe = min(p.fc_max, p.fc_sigma*sqrt(3)) * c.fc_xi(obs.k,1);
S = rl_features(c.D, obs.k, obs.Tmeas, c.af, fe);
Q = nn_lib('fwd', c.policy.net, S);
[~, a] = max(Q, [], 1);
a = rl_shield(a, obs.Tmeas, c.D, obs.k, c.levels, c.cfg);
c.af = c.levels(a);
u = c.af .* c.umax;
end
