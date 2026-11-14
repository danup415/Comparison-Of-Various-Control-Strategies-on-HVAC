function m = build_model(cfg, f)
%BUILD_MODEL  Three-zone, two-node-per-zone (air + thermal mass) RC model.
%
%   States  x = [Ta1 Ta2 Ta3 Tm1 Tm2 Tm3]'       (deg C)
%   Input   u = [Qc1 Qc2 Qc3]'                    (kW sensible cooling removed from air nodes)
%   Disturb d = [To; Tsa_roof; Tsa_wall(1:3); Qsol(1:3); Qint(1:3); Qvent(1:3)]  (deg C, W)
%
%   Air node z:  Ca dTa/dt = Ham(Tm-Ta) + (UAglz+Hinf)(To-Ta) + sum_j Hiz(Taj-Ta)
%                            + fs*Qsol + fc*Qint + Qvent - 1000*Qc
%   Mass node z: Cm dTm/dt = Ham(Ta-Tm) + UAwall(Tsa_wall-Tm) + UAroof(Tsa_roof-Tm)
%                            + (1-fs)*Qsol + (1-fc)*Qint
%
%   f (optional) scales parameters for plant/model mismatch: fields Ca, Cm, UA.
%   The model is discretised exactly (zero-order hold) at cfg.Ts.
if nargin < 2 || isempty(f), f = struct('Ca',1,'Cm',1,'UA',1); end
b = cfg.bld; rho = 1.18; cp = 1006;
nz = 3;
A_gross = b.ext_perim * b.height;              % exterior wall gross area (m2)
A_glz   = b.WWR * A_gross;
A_wall  = A_gross - A_glz;
V       = b.area * b.height;
A_part  = [b.part_len(1), sum(b.part_len), b.part_len(2)] * b.height;   % partition area seen by each zone

Ca   = f.Ca * (rho*cp*V + b.kappa_furn*b.area);
Cm   = f.Cm * ((b.kappa_roof + b.kappa_floor)*b.area + b.kappa_wall*A_wall);
Ham  = b.h_in * (2*b.area + A_wall + A_part);
UAg  = f.UA * b.U_glz * A_glz;
Hinf = f.UA * rho*cp*b.ACH_inf/3600 * V;
UAw  = f.UA * b.U_wall * A_wall;
UAr  = f.UA * b.U_roof * b.area;
H12  = b.part_len(1)*b.height / b.R_part_m2;
H23  = b.part_len(2)*b.height / b.R_part_m2;
Hiz  = [0 H12 0; H12 0 H23; 0 H23 0];

A = zeros(2*nz); B = zeros(2*nz, nz); E = zeros(2*nz, 14);
for z = 1:nz
    ia = z; im = nz + z;
    A(ia,ia) = -(Ham(z) + UAg(z) + Hinf(z) + sum(Hiz(z,:))) / Ca(z);
    A(ia,im) =  Ham(z) / Ca(z);
    for j = 1:nz
        if j ~= z, A(ia,j) = Hiz(z,j) / Ca(z); end
    end
    A(im,im) = -(Ham(z) + UAw(z) + UAr(z)) / Cm(z);
    A(im,ia) =  Ham(z) / Cm(z);
    B(ia,z)  = -1000 / Ca(z);
    E(ia,1)      = (UAg(z) + Hinf(z)) / Ca(z);          % To
    E(im,2)      = UAr(z) / Cm(z);                      % Tsa roof
    E(im,2+z)    = UAw(z) / Cm(z);                      % Tsa wall (zone-specific)
    E(ia,5+z)    = b.solar_to_air / Ca(z);              % Qsol
    E(im,5+z)    = (1-b.solar_to_air) / Cm(z);
    E(ia,8+z)    = b.int_conv / Ca(z);                  % Qint
    E(im,8+z)    = (1-b.int_conv) / Cm(z);
    E(ia,11+z)   = 1 / Ca(z);                           % Qvent (DOAS/IEC sensible)
end

% Exact ZOH discretisation
n = 2*nz; nu = nz; nd = 14; Ts = cfg.Ts;
M = zeros(n+nu+nd); M(1:n,1:n) = A; M(1:n, n+1:n+nu) = B; M(1:n, n+nu+1:end) = E;
Md = expm(M*Ts);
m.Ad = Md(1:n,1:n); m.Bd = Md(1:n,n+1:n+nu); m.Ed = Md(1:n,n+nu+1:end);
m.A = A; m.B = B; m.E = E; m.C = [eye(nz) zeros(nz)];
m.Ca = Ca; m.Cm = Cm; m.Ham = Ham; m.UAg = UAg; m.Hinf = Hinf; m.UAw = UAw; m.UAr = UAr;
m.H12 = H12; m.H23 = H23; m.V = V; m.Vtot = sum(V); m.A_glz = A_glz; m.A_wall = A_wall;
m.zone_frac = b.area / sum(b.area);
ev = eig(A);
m.tau_h = sort(-1 ./ real(ev)) / 3600;                  % modal time constants (h)
m.UA_total = UAg + Hinf + UAw + UAr;                    % W/K per zone (series path ignored)
end
