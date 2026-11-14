% Tests the Discussion's explanation of why ToD pricing produces little load shifting.
setup_paths; cfg = default_config(); ctx = setup_study(cfg); D = ctx.D; m = ctx.nm;
sel = ismember(D.month, 3:6);                         % cooling season
pk  = D.tariff == max(cfg.tariff);                     % peak windows 07-10, 18-23 (hypothetical tariff)
occ = D.occupied;
fprintf('Share of occupied zone-hours falling in peak windows (Mar-Jun): %.1f %%\n', 100*sum(sum(occ(sel & pk,:)))/sum(sum(occ(sel,:))));
fprintf('Share of all hours that are peak hours: %.1f %%\n', 100*mean(pk));
for c = {'pid','mpc','hybrid'}
  S = load(sprintf('results/runs/base__%s__s1.mat', c{1})); Ta = double(S.TS.Ta); Tm = double(S.TS.Tm); P = double(S.TS.P_vrf);
  q = (Tm - Ta) .* repmat(m.Ham, D.N, 1);                % W, heat from mass node to air node (positive = mass heats room)
  qh = arrayfun(@(h) mean(sum(q(sel & D.hod == h, :), 2)), 0:23);
  Eocc = sum(P(sel & pk & any(occ,2))) / sum(P(sel & pk));
  fprintf('%-7s mass->air heat flow, mean W by hour (Mar-Jun):\n  ', c{1}); fprintf('%5.0f', qh); fprintf('\n');
  fprintf('        peak-window VRF energy while >=1 zone occupied: %.1f %%; mean mass->air flow 18-23h: %.0f W vs 10-16h: %.0f W\n', 100*Eocc, mean(qh(19:23)), mean(qh(11:17)));
end
