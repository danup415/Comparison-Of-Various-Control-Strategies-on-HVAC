function ia = rl_shield(ia, Tmeas, D, k, levels, cfg)
%RL_SHIELD  Safety shield applied to the DQN action in training AND deployment.
%   - no cooling when the zone is within 0.5 K of the lower band, or more than 1.5 K
%     below its setpoint (prevents winter / night over-cooling);
%   - at least 50 % capacity when an occupied zone is 0.5 K above the upper band
%     (same trigger as the Hybrid controller's safety layer).
if ~cfg.ctrl.rl.shield, return; end
lo = max(D.Tlo(k,:) + 0.5, D.sp(k,:) - 1.5);
ia(Tmeas < lo) = 1;                                      % level 0
i50 = find(levels >= 0.5, 1);
hot = D.occupied(k,:) & (Tmeas > D.Thi(k,:) + cfg.ctrl.hyb.safety_dT);
ia(hot) = max(ia(hot), i50);
end
