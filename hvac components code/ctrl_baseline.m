function Q_cool = ctrl_baseline(T_avg, T_out, RH_out)
SP = 24;
Q_cool = 0;
if T_avg <= SP; return; end
dT = T_avg - SP;
vQ = 11000.0;
if T_out > 35
    Q_cool = vQ * min(1.0, max(0.2, dT/3));
elseif T_out > 28 && RH_out < 50
    Q_cool = vQ * min(0.6, max(0.2, dT/4));
elseif RH_out > 65 && T_out > 22
    Q_cool = vQ * min(0.9, max(0.2, dT/3));
elseif T_avg > SP + 0.5
    Q_cool = vQ * min(0.7, max(0.2, dT/5));
end
Q_cool = max(0, min(vQ, Q_cool));
end
