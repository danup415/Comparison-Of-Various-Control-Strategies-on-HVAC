function c = mpc_core_observe(c, obs)
%MPC_CORE_OBSERVE  Kalman update of the augmented state with the current measurement.
D = c.D; k = obs.k;
y = obs.Tmeas(:);
if isempty(c.xa)
    c.xa = [y; y + 1; zeros(3,1)];   % mass assumed ~1 K warmer than air at start
else
    xp = c.Aa*c.xa + c.Ba*c.u_last(:) + c.Ea*c.d_last;
    c.xa = xp + c.Kf*(y - c.Caug*xp);
end
c.d_last = nominal_dist(D, k, c.mdl, 0, 1);     % measured weather now, nominal gains, Qvent unknown
end

function d = nominal_dist(D, k, mdl, dTo, sol)
d = [D.To(k)+dTo; D.Tsa_roof(k)+dTo; (D.Tsa_wall(k,:)+dTo)'; sol*D.Qsol(k,:)'; D.Qint(k,:)'; zeros(3,1)];
end
