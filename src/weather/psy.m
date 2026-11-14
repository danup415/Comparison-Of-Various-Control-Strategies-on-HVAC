function out = psy(fn, varargin)
%PSY  Moist-air psychrometric helpers (SI, P in Pa, T in deg C, W in kg/kg).
%   psy('pws',T)          saturation vapour pressure (Magnus, Alduchov-Eskridge)
%   psy('w',T,RH,P)       humidity ratio from RH (%)
%   psy('rh',T,W,P)       relative humidity (%) from humidity ratio
%   psy('h',T,W)          enthalpy, kJ/kg dry air
%   psy('twb',T,RH)       wet-bulb temperature (Stull 2011, valid 5-99 %RH)
switch fn
    case 'pws'
        T = varargin{1};
        out = 610.94 .* exp(17.625 .* T ./ (T + 243.04));
    case 'w'
        [T, RH, P] = varargin{:};
        pw = RH/100 .* psy('pws', T);
        out = 0.621945 .* pw ./ (P - pw);
    case 'rh'
        [T, W, P] = varargin{:};
        pw = P .* W ./ (0.621945 + W);
        out = min(100, 100 .* pw ./ psy('pws', T));
    case 'h'
        [T, W] = varargin{:};
        out = 1.006 .* T + W .* (2501 + 1.86 .* T);
    case 'twb'
        [T, RH] = varargin{:};
        RH = min(max(RH, 5), 99);
        out = T .* atan(0.151977 .* sqrt(RH + 8.313659)) + atan(T + RH) - atan(RH - 1.676331) ...
            + 0.00391838 .* RH.^1.5 .* atan(0.023101 .* RH) - 4.686035;
    otherwise
        error('psy:fn', 'Unknown function %s', fn);
end
end
