function [u, c] = ctrl_hybrid(c, obs, mode)
%CTRL_HYBRID  Hierarchical E-MPC + PI + safety layer.
%   Layer 1 (hourly): economic MPC gives the feed-forward cooling U_h and the
%                     predicted zone-air trajectory Ta_ref at 15-min resolution.
%   Layer 2 (15 min): PI loop on the MPC prediction error e = Ta_meas - Ta_ref,
%                     u = U_h + Kc*e + v  (SIMC gain scaled by Kp_frac, back-calculation),
%                     active only while the MPC cools or the zone nears the upper band.
%                     It rejects intra-hour disturbances (cooking, solar, IEC/DOAS
%                     heat) that the hourly MPC cannot see.
%   Layer 3 (safety): if an occupied zone exceeds the upper band by safety_dT,
%                     u_z >= safety_frac * Umax_z.
if nargin > 2 && strcmp(mode, 'init')
    c = mpc_core_init(c);
    [Kc, Ti] = tune_simc(c.nm, c.cfg);
    h = c.cfg.ctrl.hyb;
    c.Kc = h.Kp_frac*Kc; c.Ti = Ti; c.v = zeros(1,3); c.jsub = 0;
    u = []; return;
end
h = c.cfg.ctrl.hyb; Ts = c.cfg.Ts; D = c.D; k = obs.k;
c = mpc_core_observe(c, obs);
if mod(k - 1, c.sub) == 0 || c.nsolve == 0
    c = mpc_core_solve(c, k); c.jsub = 0;
end
c.jsub = c.jsub + 1;
% reference: MPC-predicted air temperature at the start of this sub-step
if c.jsub == 1, Tref = c.xa(1:3)'; else, Tref = c.Ypred(c.jsub-1, :); end
e    = obs.Tmeas - Tref;
uff  = c.U(1,:);
% correct only when the MPC is cooling or the zone approaches the upper band; otherwise
% the zone is free-floating inside the band and no correction is needed
active = (uff > 0) | (obs.Tmeas > D.Thi(k,:) - c.cfg.ctrl.mpc.backoff);
e(~active) = 0; c.v(~active) = 0;
u_un = uff + c.Kc.*e + c.v;
u    = min(max(u_un, 0), c.Umax);
c.v  = c.v + Ts*(c.Kc./c.Ti.*e + (u - u_un)./c.Ti);
c.v  = 0.98*c.v;                                  % leak: MPC owns the steady state
hot  = D.occupied(k,:) & (obs.Tmeas > D.Thi(k,:) + h.safety_dT);
u(hot) = max(u(hot), h.safety_frac*c.Umax(hot));
c.u_last = u;
end
