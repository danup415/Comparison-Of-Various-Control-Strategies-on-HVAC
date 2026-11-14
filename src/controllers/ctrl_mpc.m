function [u, c] = ctrl_mpc(c, obs, mode)
%CTRL_MPC  Economic MPC (ToD-cost objective, soft comfort band, offset-free Kalman filter).
%   Re-optimised every hour; the first hourly block of the solution is applied.
if nargin > 2 && strcmp(mode, 'init')
    c = mpc_core_init(c); u = []; return;
end
c = mpc_core_observe(c, obs);
if mod(obs.k - 1, c.sub) == 0 || c.nsolve == 0
    c = mpc_core_solve(c, obs.k);
end
u = c.U(1,:);
c.u_last = u;
end
