function Q_cool = ctrl_pid(Tz1, Tz2, Tz3, h24)
persistent integ e_prev;
if isempty(integ); integ = zeros(3,1); e_prev = zeros(3,1); end
T_SET = 24; dt = 3600;
Kp   = [150.0, 250.0, 100.0];
Ki   = Kp / 7200;
Kd   = Kp * 900;
iLim = [5000, 8000, 4000];
frac = [0.3158, 0.4737, 0.2105];
db   = 0.5;
is_night = (h24 >= 23 || h24 <= 6);
SP = T_SET + 2*double(is_night);
Tz = [Tz1, Tz2, Tz3];
Q_cool = 0;
for z = 1:3
    err = Tz(z) - SP;
    if err <= db; err = 0; integ(z) = integ(z)*0.95; end
    integ(z) = max(0, min(iLim(z), integ(z)+err*dt));
    deriv = max(0, (err - e_prev(z))/dt);
    e_prev(z) = err;
    zone_cap = 11000.0 * frac(z);
    Q_z = Kp(z)*err + Ki(z)*integ(z) + Kd(z)*deriv;
    Q_z = max(0, min(zone_cap, Q_z));
    if Q_z >= zone_cap; integ(z) = integ(z) - err*dt*0.5; end
    if Q_z < 300; Q_z = 0; end
    Q_cool = Q_cool + Q_z;
end
Q_cool = max(0, min(11000.0, Q_cool));
end
