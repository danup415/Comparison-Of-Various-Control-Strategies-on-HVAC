function R = ideal_loads(ctx, cfg, sp)
%IDEAL_LOADS  Sensible cooling needed to hold each zone at or below sp (N x 3), unlimited
%   capacity, no heating, no equipment model: the same quantity EnergyPlus reports for a
%   ZoneHVAC:IdealLoadsAirSystem. Uses the nominal model (plant = model) and the exact ZOH
%   discretisation, so the cooling that brings Ta to sp at the end of each step is found by
%   solving the coupled zone equations with an active-set loop.
D = ctx.D; m = ctx.nm; N = D.N;
x = (mean(D.To(1:96)) + 3) * ones(6,1);
Ta = zeros(N,3); Q = zeros(N,3);
for k = 1:N
    d = [D.To(k); D.Tsa_roof(k); D.Tsa_wall(k,:)'; D.Qsol(k,:)'; D.Qint(k,:)'; zeros(3,1)];
    xf = m.Ad*x + m.Ed*d;  u = zeros(3,1);
    act = xf(1:3) > sp(k,:)';
    for it = 1:4
        if ~any(act), break; end
        G = m.Bd(1:3, act);                       % effect of cooling (kW) in active zones on Ta
        ua = G(act,:) \ (sp(k,act)' - xf(act));
        u = zeros(3,1); u(act) = max(0, ua);
        Tn = xf(1:3) + m.Bd(1:3,:)*u;
        newact = act | (Tn > sp(k,:)' + 1e-9);
        newact(act & u <= 0) = false;
        if isequal(newact, act), break; end
        act = newact;
    end
    x = xf + m.Bd*u;
    Ta(k,:) = x(1:3)'; Q(k,:) = u';
end
R.Ta = Ta; R.Q_kW = Q; R.E_kWh = sum(Q)*cfg.Ts/3600;
end
