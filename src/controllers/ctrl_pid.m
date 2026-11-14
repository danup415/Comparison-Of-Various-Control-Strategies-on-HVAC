function [u, c] = ctrl_pid(c, obs, mode)
%CTRL_PID  Three independent zone PI(D) loops, SIMC-tuned, back-calculation anti-windup,
%   derivative on measurement with first-order filter (N = 5). Output: sensible kW.
if nargin > 2 && strcmp(mode, 'init')
    [c.Kc, c.Ti, c.tune] = tune_simc(c.nm, c.cfg);
    p = c.cfg.ctrl.pid;
    c.Td = p.Td_frac * c.Ti;  c.Tt = p.Tt_frac * c.Ti;
    c.v = zeros(1,3); c.dfilt = zeros(1,3); c.Tprev = [];
    c.umax = c.cfg.vrf.Qiu_max * c.cfg.vrf.SHR;
    u = []; return;
end
Ts = c.cfg.Ts;
T  = obs.Tmeas; sp = c.D.sp(obs.k, :);
if isempty(c.Tprev), c.Tprev = T; end
e  = T - sp;                                     % cooling: positive error -> more cooling
Nf = 5;
alpha = c.Td ./ (c.Td + Nf*Ts + eps);
c.dfilt = alpha .* c.dfilt + (1 - alpha) .* (c.Kc .* c.Td / Ts) .* (T - c.Tprev);
c.Tprev = T;
u_un = c.Kc .* e + c.v + c.dfilt;
u    = min(max(u_un, 0), c.umax);
c.v  = c.v + Ts * (c.Kc ./ c.Ti .* e + (u - u_un) ./ c.Tt);   % back-calculation
end
