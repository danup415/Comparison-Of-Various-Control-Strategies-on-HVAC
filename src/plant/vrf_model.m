function [Qs, mw, P, Qtot, n_on] = vrf_model(v, Qcmd, To, Win)
%VRF_MODEL  Inverter VRF/multi-split: capacity, latent removal and electric power.
%   Qcmd : 1x3 commanded sensible cooling per indoor unit (kW, >= 0)
%   To   : outdoor dry-bulb (C);  Win : indoor humidity ratio (kg/kg)
%   Qs   : delivered sensible (kW); mw : moisture removed (kg/s)
%   P    : electric power incl. indoor fans (kW); Qtot : total coil load (kW)
Qcmd = max(0, Qcmd(:)');
Qs = zeros(1,3); mw = 0; P = 0; Qtot = 0; n_on = 0;
if To < v.T_oper(1) || To > v.T_oper(2) || sum(Qcmd) <= 0
    return;                                  % unit unavailable / off: no cooling, no power
end
wet = Win > v.W_coil;                        % coil removes moisture only above ADP humidity
SHR = 1; if wet, SHR = v.SHR; end
Qt  = min(Qcmd / SHR, v.Qiu_max);            % total load per indoor unit
Qcap = v.Qcap_rated * (1 - v.cap_slope * max(0, To - 35));
if sum(Qt) > Qcap, Qt = Qt * Qcap / sum(Qt); end
Qs   = Qt * SHR;
Qtot = sum(Qt);
mw   = (Qtot - sum(Qs)) / 2450;              % kW / (kJ/kg) -> kg/s
PLR  = Qtot / Qcap;
eirT = max(0.6, 1 + v.eirT_slope * (To - 35));
Pfull = Qcap / v.COP_rated * eirT;
eir  = @(x) v.eirPLR(1) + v.eirPLR(2)*x + v.eirPLR(3)*x.^2;
if PLR >= v.PLR_min
    P = Pfull * eir(PLR);
else                                         % on/off cycling below minimum modulation
    r = PLR / v.PLR_min;
    PLF = 1 - v.Cd * (1 - r);
    P = Pfull * eir(v.PLR_min) * r / PLF;
end
n_on = sum(Qt > 0);
P = P + n_on * v.P_iu_fan;
end
