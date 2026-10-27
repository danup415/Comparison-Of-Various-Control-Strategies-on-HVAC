function Q_cool = ctrl_mpc(Tz2, T_out, tariff, h24)
persistent u_prev;
if isempty(u_prev); u_prev = 0; end
T_SET = 24; Hp = 12; vrf_Qmax = 11000.0;
is_night = (h24 >= 23 || h24 <= 6);
SP = T_SET + 2*double(is_night);
if Tz2 <= SP + 0.3; Q_cool = 0; u_prev = 0; return; end
a = 0.85;
sum_e = 0; xk = Tz2;
for k = 1:Hp
    xk = a*xk + (1-a)*(T_out + 2.0);
    w_k = 0.95^(k-1);
    sum_e = sum_e + max(0, xk - SP)*w_k;
end
tod_w = 0.3 + 0.2*(tariff/7.15);
Q_demand = 800 * sum_e / Hp;
Q_demand = max(vrf_Qmax*0.20, min(vrf_Qmax*1.05, Q_demand));
if Tz2 > SP + 3.0; Q_demand = vrf_Qmax; end
du = Q_demand - u_prev;
if abs(du) > vrf_Qmax*0.30; Q_demand = u_prev + sign(du)*vrf_Qmax*0.30; end
Q_cool = max(0, min(vrf_Qmax*1.05, Q_demand));
u_prev = Q_cool;
end
