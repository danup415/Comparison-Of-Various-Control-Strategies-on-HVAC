function make_figures(cfg)
%MAKE_FIGURES  Publication figures from saved runs (results/runs) -> results/figures/*.png
%   Uses only core plotting functions so it runs in MATLAB R2019b+ and GNU Octave 8+.
if exist('OCTAVE_VERSION', 'builtin'), try, graphics_toolkit('gnuplot'); catch, end, end
rd = fullfile(cfg.out_dir, 'runs'); fd = fullfile(cfg.out_dir, 'figures');
if ~exist(fd, 'dir'), mkdir(fd); end
C = cfg.controllers; L = cfg.labels; nc = numel(C);
col = [0.45 0.45 0.45; 0.12 0.47 0.71; 0.84 0.15 0.16; 0.17 0.63 0.17; 0.58 0.40 0.74];
ctx = setup_study(scenario_config('base')); D = ctx.D; w = ctx.w;
TS = cell(nc,1); K = cell(nc,1);
for i = 1:nc
    S = load(fullfile(rd, sprintf('base__%s__s1.mat', C{i}))); TS{i} = S.TS; K{i} = S.K;
end
h = cfg.Ts/3600; spd = D.spd; N = D.N;
MON = {'J','F','M','A','M','J','J','A','S','O','N','D'};

%% Fig 1: climate (EPW)
f = figure('visible','off','position',[0 0 900 380]);
mT = accum_month(w.month, w.Tdb, @mean); mx = accum_month(w.month, w.Tdb, @max); mn = accum_month(w.month, w.Tdb, @min);
mRH = accum_month(w.month, w.RH, @mean); mG = accum_month(w.month, w.GHI, @sum)/1000;
subplot(1,2,1); hold on;
patch([1:12 12:-1:1], [mx fliplr(mn)], [0.85 0.85 0.95], 'edgecolor', 'none');
plot(1:12, mT, 'k-o', 'linewidth', 1.5); grid on; xlim([1 12]); set(gca,'xtick',1:12,'xticklabel',MON);
ylabel('Dry-bulb temperature (\circC)'); title('(a) Monthly mean and range');
subplot(1,2,2); [ax, h1, h2] = plotyy(1:12, mRH, 1:12, mG);
set(h1,'linewidth',1.5,'marker','s'); set(h2,'linewidth',1.5,'marker','o');
ylabel(ax(1),'Mean RH (%)'); ylabel(ax(2),'GHI (kWh/m^2/month)'); set(ax,'xtick',1:12,'xticklabel',MON,'xlim',[1 12]);
title('(b) Humidity and solar'); grid on;
save_fig(f, fd, 'Fig01_climate');

%% Fig 2: design weeks (hottest week from EPW header: 6-12 May; monsoon week 1-7 Aug)
weeks = {[126 7], 'Pre-monsoon extreme week (6-12 May)'; [213 7], 'Monsoon week (1-7 Aug)'};
for wk = 1:2
    f = figure('visible','off','position',[0 0 1000 620]);
    d0 = weeks{wk,1}(1); ks = (d0-1)*spd + (1:weeks{wk,1}(2)*spd); t = (ks - ks(1))*h/24;
    subplot(3,1,1); hold on;
    occ = D.occupied(ks,2);
    band_patch(t, D.Tlo(ks,2), D.Thi(ks,2));
    for i = 1:nc, plot(t, TS{i}.Ta(ks,2), 'color', col(i,:), 'linewidth', 1.1); end
    plot(t, D.To(ks), 'k:', 'linewidth', 1);
    ylabel('Zone 2 air (\circC)'); grid on; title(weeks{wk,2});
    legend([{'Comfort band'} L {'Outdoor'}], 'location', 'eastoutside');
    subplot(3,1,2); hold on;
    band_patch(t, D.Tlo(ks,1), D.Thi(ks,1));
    for i = 1:nc, plot(t, TS{i}.Ta(ks,1), 'color', col(i,:), 'linewidth', 1.1); end
    ylabel('Zone 1 air (\circC)'); grid on; legend([{'Comfort band'} L], 'location', 'eastoutside');
    subplot(3,1,3); hold on;
    tr = D.tariff(ks); pk = tr == max(tr);
    ytop = 0; for i = 1:nc, ytop = max(ytop, max(TS{i}.P(ks))); end; ytop = 1.05*ytop;
    area_flags(t, pk, ytop);                                 % shading first (opaque in gnuplot)
    hl = zeros(1,nc); for i = 1:nc, hl(i) = stairs(t, TS{i}.P(ks), 'color', col(i,:), 'linewidth', 1); end
    ylim([0 ytop]); xlim([0 t(end)]);
    ylabel('Electric power (kW)'); xlabel('Day (shaded: peak tariff)'); grid on; legend(hl, L, 'location', 'eastoutside');
    save_fig(f, fd, sprintf('Fig02%s_week_%d', char('a'+wk-1), wk));
end

%% Fig 3: monthly energy (real monthly sums) and annual end use
f = figure('visible','off','position',[0 0 1000 400]);
Em = zeros(12, nc); mon = D.month;
for i = 1:nc, for m = 1:12, Em(m,i) = sum(TS{i}.P(mon == m))*h; end, end
subplot(1,2,1); b = bar(Em, 'grouped'); for i = 1:nc, set(b(i), 'facecolor', col(i,:)); end
set(gca,'xtick',1:12,'xticklabel',MON); ylabel('Electricity (kWh/month)'); grid on; legend(L, 'location','northeast');
title('(a) Monthly electricity');
subplot(1,2,2);
EU = zeros(nc,4); for i = 1:nc, EU(i,:) = [K{i}.E_vrf K{i}.E_iec K{i}.E_doas K{i}.E_filt]; end
b = bar(EU, 'stacked'); cc = [0.2 0.4 0.8; 0.1 0.7 0.4; 0.9 0.5 0.1; 0.6 0.6 0.6];
for j = 1:4, set(b(j), 'facecolor', cc(j,:)); end
set(gca,'xtick',1:nc,'xticklabel',L); ylabel('Annual electricity (kWh)'); grid on;
legend({'VRF','IEC','DOAS','Filtration fan'}, 'location','eastoutside'); title('(b) End use');
save_fig(f, fd, 'Fig03_energy');

%% Fig 4: mean daily profile of VRF power vs ToD tariff (cooling season Mar-Jun)
f = figure('visible','off','position',[0 0 700 400]); hold on;
sel = ismember(mon, 3:6); hod = D.hod;
prof = zeros(24, nc);
for i = 1:nc, for hh = 0:23, prof(hh+1,i) = mean(TS{i}.P_vrf(sel & hod == hh)); end, end
ytop = 1.1*max(prof(:));
area_flags(0:23, cfg.tariff' == max(cfg.tariff), ytop);   % shading first
hl = zeros(1,nc);
for i = 1:nc, hl(i) = plot(0:23, prof(:,i), '-o', 'color', col(i,:), 'linewidth', 1.6, 'markersize', 3); end
xlabel('Hour of day (shaded: peak tariff)'); ylabel('Mean VRF power, Mar-Jun (kW)'); xlim([0 23]); ylim([0 ytop]); grid on;
legend(hl, L, 'location','northwest');
save_fig(f, fd, 'Fig04_daily_profile');

%% Fig 5: Pareto cost vs discomfort with seed CIs
Ag = load(fullfile(cfg.out_dir, 'aggregate.mat')); Ag = Ag.A;
f = figure('visible','off','position',[0 0 620 460]); hold on;
iC = find(strcmp(Ag.kp, 'cost')); iK = find(strcmp(Ag.kp, 'Kh_total'));
for i = 1:nc
    x = Ag.mean(i,iC)/1000; y = Ag.mean(i,iK);
    plot([x x], y + [-1 1]*Ag.hw(i,iK), 'color', col(i,:), 'linewidth', 1.5);
    plot(x + [-1 1]*Ag.hw(i,iC)/1000, [y y], 'color', col(i,:), 'linewidth', 1.5);
    plot(x, y, 'o', 'markerfacecolor', col(i,:), 'markeredgecolor', 'k', 'markersize', 9);
    text(x, y, ['  ' L{i}], 'fontsize', 10);
end
xlabel('Annual electricity cost (INR thousand, ToD tariff)'); ylabel('Occupied discomfort (K h/yr)');
grid on; title('Cost-comfort trade-off (mean \pm 95% CI, 5 seeds)');
save_fig(f, fd, 'Fig05_pareto');

%% Fig 6: comfort by zone and temperature distributions (occupied)
f = figure('visible','off','position',[0 0 1000 400]);
subplot(1,2,1); cz = zeros(nc,3); for i = 1:nc, cz(i,:) = K{i}.comfort_pct_zone; end
b = bar(cz'); for i = 1:nc, set(b(i), 'facecolor', col(i,:)); end
set(gca,'xticklabel',{'Z1 bedroom','Z2 living','Z3 bedroom'}); ylabel('Occupied hours in band (%)'); ylim([0 100]); grid on;
legend(L,'location','southoutside','orientation','horizontal'); title('(a) Comfort by zone');
subplot(1,2,2); hold on; edges = 18:0.25:32; ctr = edges(1:end-1) + 0.125;
for i = 1:nc
    T = TS{i}.Ta(D.occupied); c = histc(T, edges); c = c(1:end-1);
    plot(ctr, 100*c/sum(c), 'color', col(i,:), 'linewidth', 1.4);
end
yl = ylim; plot([23 23], yl, 'k--'); plot([26 26], yl, 'k--');
xlabel('Occupied zone air temperature (\circC)'); ylabel('Share of occupied zone-hours (%)'); grid on; legend(L); title('(b) Distribution');
save_fig(f, fd, 'Fig06_comfort');

%% Fig 7: IAQ scenarios (daily means)
scn = {'base','iaq_nofilter','vent_humidity','vent_off'};
lab = {'Continuous DOAS + HEPA recirc.','No recirc. filter','Humidity-triggered DOAS (original)','No mechanical ventilation'};
f = figure('visible','off','position',[0 0 1000 560]);
cc = [0 0.45 0.74; 0.85 0.33 0.10; 0.93 0.69 0.13; 0.49 0.18 0.56];
for s = 1:numel(scn)
    fn = fullfile(rd, sprintf('%s__hybrid__s1.mat', scn{s}));
    if ~exist(fn,'file'), continue; end
    S = load(fn); nd = floor(N/spd);
    co = mean(reshape(double(S.TS.CO2(1:nd*spd)), spd, nd), 1);
    pmv = mean(reshape(double(S.TS.PM(1:nd*spd)), spd, nd), 1);
    subplot(1,2,1); hold on; plot(1:nd, co, 'color', cc(s,:), 'linewidth', 1.2);
    subplot(1,2,2); hold on; plot(1:nd, pmv, 'color', cc(s,:), 'linewidth', 1.2);
end
PMo = mean(reshape(D.PMout(1:nd*spd), spd, nd), 1);
subplot(1,2,1); ylabel('Daily mean CO_2 (ppm)'); xlabel('Day of year'); grid on; legend(lab, 'location','southoutside'); title('(a) CO_2 (base and no-filter curves coincide)');
subplot(1,2,2); plot(1:nd, PMo, 'k:', 'linewidth', 1); plot([1 nd], [15 15], 'r--'); plot([1 nd],[60 60],'k--');
ylabel('Daily mean PM_{2.5} (\mug/m^3)'); xlabel('Day of year'); grid on; title('(b) PM_{2.5}');
legend([lab {'Outdoor','WHO 24-h (15)','NAAQS 24-h (60)'}], 'location','southoutside');
save_fig(f, fd, 'Fig07_iaq');

%% Fig 8: sensitivity of annual cost and discomfort
scn = {'base','heatwave','mass_low','mass_high','flat_tariff','no_mismatch'};
sl  = {'Base','+3 K','Mass/2','Mass x2','Flat','No mism.'};
cost = nan(numel(scn), nc); khs = cost;
for s = 1:numel(scn), for i = 1:nc
    fn = fullfile(rd, sprintf('%s__%s__s1.mat', scn{s}, C{i}));
    if exist(fn,'file'), S = load(fn); cost(s,i) = S.K.cost; khs(s,i) = S.K.Kh_total; end
end, end
f = figure('visible','off','position',[0 0 1000 420]);
subplot(1,2,1); b = bar(100*(cost - repmat(cost(:,1),1,nc))./repmat(cost(:,1),1,nc));
for i = 1:nc, set(b(i),'facecolor',col(i,:)); end
set(gca,'xticklabel',sl); ylabel('Cost vs thermostat (%)'); grid on; legend(L,'location','southoutside','orientation','horizontal'); title('(a) Cost');
subplot(1,2,2); b = bar(khs); for i = 1:nc, set(b(i),'facecolor',col(i,:)); end
set(gca,'xticklabel',sl); ylabel('Occupied discomfort (K h/yr)'); grid on; title('(b) Discomfort');
save_fig(f, fd, 'Fig08_sensitivity');

%% Fig 9: DQN learning curves (validation return, all seeds)
f = figure('visible','off','position',[0 0 620 400]); hold on; lg = {};
for s = cfg.seeds
    pf = fullfile(cfg.out_dir, 'policies', sprintf('dqn_base_s%d.mat', s));
    if ~exist(pf,'file'), continue; end
    P = load(pf); vh = P.pol.val_hist; plot(vh(:,1), vh(:,2), '-o', 'linewidth', 1.2, 'markersize', 3);
    lg{end+1} = sprintf('seed %d', s); %#ok<AGROW>
end
xlabel('Training episode (14 days each)'); ylabel('Validation return'); grid on; legend(lg, 'location','southeast');
title('DQN validation return (greedy, perturbed weather)');
save_fig(f, fd, 'Fig09_dqn_learning');
end

%% helpers
function v = accum_month(m, x, fn)
v = zeros(1,12); for k = 1:12, v(k) = fn(x(m == k)); end
end
function band_patch(t, lo, hi)
t = t(:)'; patch([t fliplr(t)], [lo(:)' fliplr(hi(:)')], [0.88 0.95 0.88], 'edgecolor', 'none');
end
function area_flags(t, flag, ytop)
t = t(:); flag = flag(:);
d = diff([0; flag; 0]); s = find(d == 1); e = find(d == -1) - 1;
for j = 1:numel(s)
    x0 = t(s(j)); x1 = t(min(e(j)+1, numel(t)));
    patch([x0 x1 x1 x0], [0 0 ytop ytop], [1 0.85 0.6], 'edgecolor', 'none', 'facealpha', 0.35);
end
end
function save_fig(f, fd, name)
set(f, 'paperpositionmode', 'auto');
print(f, fullfile(fd, [name '.png']), '-dpng', '-r200');
close(f);
end
