function S = rl_features(D, k, Tmeas, ufrac_prev, fc_err)
%RL_FEATURES  15-dimensional per-zone state for the shared DQN (one column per zone).
%   [ (Ta - sp)/2, (To - 30)/8, (To_fc(+3h) - To)/4, sin(2*pi*h/24), cos(2*pi*h/24),
%     (tariff - 7.15)/1.9, occupied_now, occupied_in_1h, (sp - 24.5)/4,
%     previous action fraction, sin/cos(day of year), zone one-hot (3) ]
%   Every feature is observable in a real home (thermostat reading, weather
%   forecast, clock, tariff schedule, occupancy schedule). The night/occupancy
%   setpoint is an explicit input, unlike the original study.
N = D.N; spd = D.spd;
k3 = mod(k + 3*spd/24 - 1, N) + 1;  k1 = mod(k + spd/24 - 1, N) + 1;
To = D.To(k); Tf = D.To(k3) + fc_err;
h  = (D.hod(k) + 0.5) * 2*pi/24;
S = zeros(15, 3); dy = 2*pi*(D.doy(k) - 0.5)/365;
for z = 1:3
    sp = D.sp(k,z);
    S(:,z) = [ (Tmeas(z) - sp)/2; (To - 30)/8; (Tf - To)/4; sin(h); cos(h); ...
               (D.tariff(k) - 7.15)/1.9; D.occupied(k,z); D.occupied(k1,z); (sp - 24.5)/4; ...
               ufrac_prev(z); sin(dy); cos(dy); (1:3)' == z ];
end
end
