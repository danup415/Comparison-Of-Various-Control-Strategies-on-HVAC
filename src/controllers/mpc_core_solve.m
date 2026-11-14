function c = mpc_core_solve(c, k)
%MPC_CORE_SOLVE  Solve the economic MPC problem at step k (see mpc_core_init).
cfg = c.cfg; p = cfg.ctrl.mpc; v = cfg.vrf; D = c.D;
Np = c.Np; Nh = c.Nh; sub = c.sub; nu = 3;
tic_s = tic;
idx = mod((k:k+Np-1) - 1, D.N) + 1;             % circular year
idy = mod((k+1:k+Np) - 1, D.N) + 1;             % output instants
% ---- forecast (persistence-free: true series + growing correlated error)
if p.perfect
    eT = zeros(Np,1); sol = ones(Np,1);
else
    hrs = (1:Np)' * cfg.Ts/3600;
    eT  = min(p.fc_max, p.fc_sigma*sqrt(hrs)) * c.fc_xi(k,1);
    sol = max(0, 1 + p.sol_err*c.fc_xi(k,2)) * ones(Np,1);
end
gs = 1; gi = 1;
if p.perfect, gs = c.pm.gain_solar; gi = c.pm.gain_int; end
To_fc = D.To(idx) + eT;
% ---- free response
xa = c.xa; x = xa(1:6); dq = xa(7:9);
Yf = zeros(Np, 3);
for j = 1:Np
    kk = idx(j);
    d = [To_fc(j); D.Tsa_roof(kk)+eT(j); (D.Tsa_wall(kk,:)+eT(j))'; gs*sol(j)*D.Qsol(kk,:)'; ...
         gi*D.Qint(kk,:)'; zeros(3,1)];
    x = c.mdl.Ad*x - c.mdl.Bd*dq + c.mdl.Ed*d;
    Yf(j,:) = x(1:3)';
end
yf = reshape(Yf', [], 1);
Tlo = reshape(D.Tlo(idy,:)', [], 1) + p.backoff;
Thi = reshape(D.Thi(idy,:)', [], 1) - p.backoff;
% ---- cost per kW per hour block (INR); COP from the forecast outdoor temperature
To_h = mean(reshape(To_fc, sub, Nh), 1)';
tr_h = D.tariff(idx(1:sub:end));
eirT = max(0.6, 1 + v.eirT_slope*(To_h - 35));
cvec = tr_h .* eirT / v.COP_rated / v.SHR * (sub*cfg.Ts/3600);       % INR per kW-sensible per block
Cm   = repmat(cvec, 1, nu);                                          % Nh x nu
cap  = v.Qcap_rated * (1 - v.cap_slope*max(0, To_h - 35)) * v.SHR;   % Nh x 1
% ---- FISTA
U  = [c.Uwarm(2:end,:); c.Uwarm(end,:)];
U  = project_rows(U, c.Umax, cap);
Yv = U; t = 1; L = c.L; rho = p.rho; lam = p.lambda; u0 = c.u_last;
for it = 1:p.iters
    yy = yf + c.G*reshape(Yv', [], 1);
    gy = 2*rho*(max(0, yy - Thi) - max(0, Tlo - yy));
    g  = reshape(c.Gt*gy, nu, Nh)' + Cm;
    dU = diff([u0; Yv], 1, 1);                              % Nh x nu
    g  = g + 2*lam*dU - 2*lam*[dU(2:end,:); zeros(1,nu)];
    Un = project_rows(Yv - g/L, c.Umax, cap);
    tn = (1 + sqrt(1 + 4*t^2))/2;
    Yv = Un + ((t - 1)/tn)*(Un - U);
    U = Un; t = tn;
end
c.U = U; c.Uwarm = U;
c.Ypred = reshape(yf + c.G*reshape(U', [], 1), nu, Np)';
c.t_solve = c.t_solve + toc(tic_s); c.nsolve = c.nsolve + 1;
end

function U = project_rows(U, ub, cap)
% Euclidean projection of each row onto {0 <= u <= ub, sum(u) <= cap}
U = min(max(U, 0), repmat(ub, size(U,1), 1));
over = sum(U, 2) > cap;
if any(over)
    V = U(over,:) ; cp = cap(over);
    % find mu >= 0 with sum(clip(v - mu, 0, ub)) = cap, by bisection
    lo = zeros(size(cp)); hi = max(V, [], 2);
    for it = 1:40
        mu = (lo + hi)/2;
        s = sum(min(max(V - mu, 0), repmat(ub, size(V,1), 1)), 2);
        lo(s > cp) = mu(s > cp); hi(s <= cp) = mu(s <= cp);
    end
    U(over,:) = min(max(V - hi, 0), repmat(ub, size(V,1), 1));
end
end
