function T = run_energyplus_comparison(eplus_exe)
%RUN_ENERGYPLUS_COMPARISON  Run EnergyPlus on the equivalent model and compare with the RC model.
%   eplus_exe: path to energyplus executable, e.g.
%     'C:\EnergyPlusV24-2-0\energyplus.exe'  (Windows)  or  '/usr/local/EnergyPlus-24-2-0/energyplus'
%   Writes validation/energyplus/comparison.csv and a figure; prints the metrics:
%     free-floating zone air temperature: hourly mean bias, RMSE, R^2 (all hours)
%     ideal sensible cooling: annual kWh and monthly CV(RMSE)/NMBE (ASHRAE Guideline 14 style)
here = fileparts(mfilename('fullpath')); cdir = fullfile(here, 'case');
if ~exist(fullfile(cdir, 'pune_free.idf'), 'file'), make_energyplus_case(cdir); end
for mode = {'free', 'ideal'}
    od = fullfile(cdir, ['out_' mode{1}]); if ~exist(od, 'dir'), mkdir(od); end
    cmd = sprintf('"%s" -x -r -w "%s" -d "%s" "%s"', eplus_exe, fullfile(cdir, 'weather.epw'), od, ...
                  fullfile(cdir, ['pune_' mode{1} '.idf']));
    old = cd(cdir); [st, msg] = system(cmd); cd(old);          % schedule CSVs are found relative to cdir
    if st ~= 0, error('EnergyPlus failed (%s):\n%s', mode{1}, msg); end
end
cfg = validation_config(); ctx = setup_study(cfg); D = ctx.D; N = D.N;
% ---- RC model results
Ro = simulate(ctx, cfg, 'off', 1);  Ro_Ta = Ro.Ta;
cfg0 = cfg; cfg0.sensor_noise_std = 0;
Ri = ideal_loads(ctx, cfg0, D.sp);
% ---- EnergyPlus results
Ef = readtable(fullfile(cdir, 'out_free', 'eplusout.csv'), 'VariableNamingRule', 'preserve');
Ei = readtable(fullfile(cdir, 'out_ideal', 'eplusout.csv'), 'VariableNamingRule', 'preserve');
zn = {'Z1','Z2','Z3'};
Tf = zeros(N,3); Qe = zeros(N,3);
for z = 1:3
    Tf(:,z) = Ef{1:N, find(contains(Ef.Properties.VariableNames, [zn{z} ':Zone Mean Air Temperature']), 1)};
    Qe(:,z) = Ei{1:N, find(contains(Ei.Properties.VariableNames, [zn{z} ' IDEAL LOADS AIR SYSTEM:Zone Ideal Loads Zone Sensible Cooling Rate'], 'IgnoreCase', true), 1)} / 1000;
end
% hourly means
hm = @(X) squeeze(mean(reshape(X, 4, N/4, []), 1));
Th_rc = hm(Ro_Ta); Th_ep = hm(Tf);
e = Th_rc - Th_ep;
bias = mean(e); rmse = sqrt(mean(e.^2));
r2 = 1 - sum(e.^2) ./ sum((Th_ep - mean(Th_ep)).^2);
E_rc = sum(Ri.Q_kW)*0.25; E_ep = sum(Qe)*0.25;
mon = D.month; Mrc = zeros(12,1); Mep = zeros(12,1);
for m = 1:12, Mrc(m) = sum(sum(Ri.Q_kW(mon == m,:)))*0.25; Mep(m) = sum(sum(Qe(mon == m,:)))*0.25; end
nmbe = 100*sum(Mep - Mrc)/(12*mean(Mep)); cvrmse = 100*sqrt(mean((Mep - Mrc).^2))/mean(Mep);
fprintf('\nFree-floating zone air temperature, RC model vs EnergyPlus (hourly, full year)\n');
for z = 1:3, fprintf('  %s: bias %+.2f K, RMSE %.2f K, R^2 %.3f\n', zn{z}, bias(z), rmse(z), r2(z)); end
fprintf('Ideal sensible cooling (cooling-only, same setpoint schedule)\n');
for z = 1:3, fprintf('  %s: RC %.0f kWh, EnergyPlus %.0f kWh (%+.1f %%)\n', zn{z}, E_rc(z), E_ep(z), 100*(E_rc(z)-E_ep(z))/E_ep(z)); end
fprintf('  Total: RC %.0f kWh, EnergyPlus %.0f kWh (%+.1f %%); monthly NMBE %.1f %%, CV(RMSE) %.1f %%\n', ...
        sum(E_rc), sum(E_ep), 100*(sum(E_rc)-sum(E_ep))/sum(E_ep), nmbe, cvrmse);
fprintf('  (ASHRAE Guideline 14 monthly calibration criteria: |NMBE| <= 5 %%, CV(RMSE) <= 15 %%)\n');
T = table(zn', bias', rmse', r2', E_rc', E_ep', 'VariableNames', {'zone','T_bias_K','T_rmse_K','T_R2','Ecool_RC_kWh','Ecool_EP_kWh'});
writetable(T, fullfile(here, 'comparison.csv'));
f = figure('Visible', 'off', 'Position', [0 0 1000 420]);
subplot(1,2,1); k = (125*24+1):(132*24); plot(k/24, Th_ep(k,2), 'k', k/24, Th_rc(k,2), 'r--', 'LineWidth', 1.2);
xlabel('Day of year'); ylabel('Zone 2 free-floating air temp. (\circC)'); legend('EnergyPlus', 'RC model'); grid on;
subplot(1,2,2); bar([Mep Mrc]); legend('EnergyPlus', 'RC model'); xlabel('Month'); ylabel('Ideal sensible cooling (kWh)'); grid on;
print(f, fullfile(here, 'energyplus_comparison.png'), '-dpng', '-r200'); close(f);
end
