function [u, c] = ctrl_off(c, obs, mode)
%CTRL_OFF  Free-floating reference (no cooling). Used only to separate weather-driven
%   cold discomfort (no heating is installed) from cooling-induced overcooling.
u = zeros(1,3);
end
