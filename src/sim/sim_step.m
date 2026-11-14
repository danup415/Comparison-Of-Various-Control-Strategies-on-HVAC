function [E, u, lg] = sim_step(E, k)
%SIM_STEP  One closed-loop step: noisy measurement -> controller -> plant.
obs.k = k; obs.Tmeas = E.s.x(1:3)' + E.noise(k,:);
[u, E.c] = E.f(E.c, obs);
[E.s, lg] = plant_step(E.s, u, k, E.D, E.pm, E.cfg);
end
