function c = mpc_core_init(c)
%MPC_CORE_INIT  Economic MPC set-up: prediction matrices, Kalman filter, solver constants.
%
%   Problem solved every Ts_mpc (1 h), inputs held over each hour (move blocking):
%     min_U  sum_h c_h' U_h                                     (ToD electricity cost, INR)
%          + rho * sum_j [ max(0, Ta_j - Thi_j)^2 + max(0, Tlo_j - Ta_j)^2 ]   (soft comfort)
%          + lambda * sum_h || U_h - U_{h-1} ||^2               (move suppression)
%     s.t. 0 <= U_hz <= Umax_z,   sum_z U_hz <= Qcap(To_h) * SHR
%   Ta_j = Ta_free_j + Gamma*U  from the linear 6-state model; Ta_free uses the Kalman
%   estimate (augmented with 3 integrating heat-disturbance states for offset-free
%   prediction) and the weather/solar forecast. Solved with FISTA + exact projection.
cfg = c.cfg; p = cfg.ctrl.mpc;
if p.perfect, mdl = c.pm; else, mdl = c.nm; end
c.mdl = mdl;
c.sub = round(cfg.Ts_mpc / cfg.Ts);
c.Nh  = p.N_h; c.Np = c.Nh * c.sub;
nx = 6; nu = 3;
% ---- prediction matrix Gamma (outputs Ta at steps 1..Np, inputs blocked per hour)
G = zeros(c.Np*nu, c.Nh*nu);
for h = 1:c.Nh
    for z = 1:nu
        x = zeros(nx,1); col = (h-1)*nu + z;
        for j = 1:c.Np
            uj = zeros(nu,1);
            if ceil(j / c.sub) == h, uj(z) = 1; end
            x = mdl.Ad*x + mdl.Bd*uj;
            G((j-1)*nu+(1:nu), col) = x(1:3);
        end
    end
end
c.G = G; c.Gt = G';
c.L = 2*p.rho*norm(G)^2 + 8*p.lambda;
% ---- steady-state Kalman filter on augmented model [x; dq]
Bq = -mdl.Bd;                                     % +1 kW heat disturbance on each air node
Aa = [mdl.Ad Bq; zeros(3,6) eye(3)];
Ca = [eye(3) zeros(3,6)];
Qn = diag([0.02^2*ones(1,3), 0.01^2*ones(1,3), 0.05^2*ones(1,3)]);
Rn = max(cfg.sensor_noise_std, 0.02)^2 * eye(3);
P = eye(9);
for it = 1:2000
    Pp = Aa*P*Aa' + Qn;
    K  = Pp*Ca' / (Ca*Pp*Ca' + Rn);
    Pn = (eye(9) - K*Ca)*Pp;
    if norm(Pn - P, 'fro') < 1e-12, P = Pn; break; end
    P = Pn;
end
c.Kf = K; c.Aa = Aa; c.Ba = [mdl.Bd; zeros(3)]; c.Ea = [mdl.Ed; zeros(3,14)]; c.Caug = Ca;
c.xa = [];                         % initialised at first observation
c.u_last = zeros(1,3); c.d_last = [];
c.Umax = cfg.vrf.Qiu_max * cfg.vrf.SHR;
c.Uwarm = zeros(c.Nh, nu);
c.U = zeros(c.Nh, nu); c.Ypred = []; c.t_solve = 0; c.nsolve = 0;
end
