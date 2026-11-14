function [u, c] = ctrl_thermostat(c, obs, mode)
%CTRL_THERMOSTAT  Baseline: per-zone on/off thermostat with hysteresis (typical split AC).
%   Indoor unit runs at full sensible capacity when on. Same occupancy-based
%   setpoint schedule (D.sp) as every other controller.
if nargin > 2 && strcmp(mode, 'init')
    c.on = false(1,3);
    c.umax = c.cfg.vrf.Qiu_max * c.cfg.vrf.SHR;
    u = []; return;
end
h  = c.cfg.ctrl.thermo.hyst;
sp = c.D.sp(obs.k, :);
T  = obs.Tmeas;
c.on(T > sp + h) = true;
c.on(T < sp - h) = false;
u = c.umax .* c.on;
end
