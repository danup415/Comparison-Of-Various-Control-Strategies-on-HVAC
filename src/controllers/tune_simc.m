function [Kc, Ti, info] = tune_simc(nm, cfg)
%TUNE_SIMC  Identify a first-order-plus-dead-time model of each zone (air temperature
%   response to a 1 kW sensible cooling step, from the controller's nominal model)
%   and tune PI gains with the SIMC rules (Skogestad 2003).
%   Kc in kW/K, Ti in s.
Ts = cfg.Ts; nsteps = round(48*3600/Ts);
tauc = cfg.ctrl.pid.tau_c;
Kc = zeros(1,3); Ti = zeros(1,3); info = struct('K',[],'tau',[],'theta',[]);
for z = 1:3
    x = zeros(6,1); u = zeros(3,1); u(z) = 1; y = zeros(nsteps,1);
    for k = 1:nsteps, x = nm.Ad*x + nm.Bd*u; y(k) = -x(z); end   % K drop per kW
    K = y(end);
    t = (1:nsteps)'*Ts;
    k63 = find(y >= 0.632*K, 1);
    tau = t(k63);
    theta = Ts/2;                                % sampling-induced effective delay
    % Two-time-scale plant: use the initial (air-node) slope as an integrating
    % approximation, k' = dy/dt at t=0, which is the SIMC rule for lag-dominant processes.
    kp = y(1)/Ts;                                % K/s per kW
    Kc(z) = 1/(kp*(tauc + theta));
    Ti(z) = min(tau, 4*(tauc + theta));
    info.K(z) = K; info.tau(z) = tau; info.theta(z) = theta; info.kprime(z) = kp;
end
end
