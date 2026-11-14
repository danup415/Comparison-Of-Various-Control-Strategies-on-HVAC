function U = analyze_uq(cfg, nS)
%ANALYZE_UQ  Robustness of the controller comparison to every [ASSUMED] input.
%   Reads results/runs/uq<i>__<ctrl>__s1.mat (40 Latin-hypercube samples of U-values,
%   glazing, infiltration, VRF COP and part-load curve, occupancy, internal gains, thermal
%   mass, cooking leakage) and reports, across samples:
%     - how often each controller has the lowest cost / discomfort among the modulating ones
%     - distribution of Hybrid-vs-PID and Hybrid-vs-E-MPC differences
%     - Spearman rank correlation of each input with the Hybrid-vs-PID cost difference
%   Writes results/tables/uq_summary.csv and uq_samples.csv.
if nargin < 2, nS = 40; end
C = cfg.controllers; L = cfg.labels; nc = numel(C);
rd = fullfile(cfg.out_dir, 'runs'); td = fullfile(cfg.out_dir, 'tables');
cost = nan(nS, nc); kh = nan(nS, nc); E = nan(nS, nc); X = [];
for i = 1:nS
    [~, x, names] = uq_config(i); X(i,:) = x; %#ok<AGROW>
    for c = 1:nc
        fn = fullfile(rd, sprintf('uq%d__%s__s1.mat', i, C{c}));
        if exist(fn, 'file'), S = load(fn); cost(i,c) = S.K.cost; kh(i,c) = S.K.Kh_total; E(i,c) = S.K.E_total; end
    end
end
ok = all(~isnan(cost), 2); n = sum(ok);
cost = cost(ok,:); kh = kh(ok,:); X = X(ok,:);
U.n = n;
mod = 2:nc;                                            % modulating controllers (exclude thermostat)
[~, bc] = min(cost(:,mod), [], 2); [~, bk] = min(kh(:,mod), [], 2);
fid = fopen(fullfile(td, 'uq_summary.csv'), 'w');
fprintf(fid, 'samples,%d\n', n);
fprintf(fid, 'controller,share_lowest_cost_among_modulating,share_lowest_discomfort_among_modulating,median_cost,median_Kh\n');
fprintf('\nUncertainty analysis over %d building/equipment samples\n', n);
for j = 1:numel(mod)
    sc = 100*mean(bc == j); sk = 100*mean(bk == j);
    fprintf(fid, '%s,%.1f,%.1f,%.0f,%.0f\n', L{mod(j)}, sc, sk, median(cost(:,mod(j))), median(kh(:,mod(j))));
    fprintf('  %-7s lowest cost in %5.1f %% of samples, lowest discomfort in %5.1f %%\n', L{mod(j)}, sc, sk);
end
iH = find(strcmp(C,'hybrid')); iP = find(strcmp(C,'pid')); iM = find(strcmp(C,'mpc')); iT = find(strcmp(C,'thermostat'));
dHP = 100*(cost(:,iH) - cost(:,iP))./cost(:,iP); kHP = kh(:,iH) - kh(:,iP);
dHM = 100*(cost(:,iH) - cost(:,iM))./cost(:,iM); kHM = kh(:,iH) - kh(:,iM);
dom_HP = mean(dHP < 0 & kHP < 0); dom_HM = mean(dHM < 0 & kHM < 0);
thermo_cheapest = mean(cost(:,iT) < min(cost(:,mod), [], 2));
thermo_worst_kh = mean(kh(:,iT) > max(kh(:,mod), [], 2));
q = @(v) prctile_(v, [5 50 95]);
fprintf('  Hybrid vs PID cost: 5th/50th/95th pct %+.1f / %+.1f / %+.1f %%; Hybrid better on BOTH cost and discomfort in %.0f %% of samples\n', q(dHP), 100*dom_HP);
fprintf('  Hybrid vs E-MPC cost: 5th/50th/95th pct %+.1f / %+.1f / %+.1f %%; Hybrid better on both in %.0f %%\n', q(dHM), 100*dom_HM);
fprintf('  Thermostat cheapest in %.0f %% of samples; worst discomfort in %.0f %%\n', 100*thermo_cheapest, 100*thermo_worst_kh);
fprintf(fid, 'hybrid_vs_pid_cost_pct_p5_p50_p95,%.2f,%.2f,%.2f\n', q(dHP));
fprintf(fid, 'hybrid_vs_pid_Kh_p5_p50_p95,%.1f,%.1f,%.1f\n', q(kHP));
fprintf(fid, 'hybrid_dominates_pid_share,%.3f\n', dom_HP);
fprintf(fid, 'hybrid_vs_mpc_cost_pct_p5_p50_p95,%.2f,%.2f,%.2f\n', q(dHM));
fprintf(fid, 'hybrid_vs_mpc_Kh_p5_p50_p95,%.1f,%.1f,%.1f\n', q(kHM));
fprintf(fid, 'hybrid_dominates_mpc_share,%.3f\n', dom_HM);
fprintf(fid, 'thermostat_cheapest_share,%.3f\nthermostat_worst_discomfort_share,%.3f\n', thermo_cheapest, thermo_worst_kh);
% Spearman correlation of inputs with Hybrid-vs-PID cost difference
fprintf(fid, 'input,spearman_with_hybrid_minus_pid_cost_pct\n'); fprintf('  Spearman rho of inputs with Hybrid-vs-PID cost difference:\n');
for j = 1:size(X,2)
    rho = spearman_(X(:,j), dHP); fprintf(fid, '%s,%.2f\n', names{j}, rho); fprintf('    %-12s %+.2f\n', names{j}, rho);
end
fclose(fid);
fid = fopen(fullfile(td, 'uq_samples.csv'), 'w');
fprintf(fid, '%s,', names{:}); fprintf(fid, 'cost_%s,', C{:}); fprintf(fid, 'Kh_%s,', C{1:end-1}); fprintf(fid, 'Kh_%s\n', C{end});
fprintf(fid, [repmat('%.4g,', 1, numel(names) + 2*nc - 1) '%.4g\n'], [X cost kh]');
fclose(fid);
U.cost = cost; U.kh = kh; U.X = X; U.names = names; U.dHP = dHP; U.kHP = kHP; U.dHM = dHM; U.kHM = kHM;
end

function p = prctile_(v, q)
v = sort(v(:)); n = numel(v); p = zeros(size(q));
for i = 1:numel(q), p(i) = v(max(1, min(n, round(q(i)/100*(n-1)) + 1))); end
end
function r = spearman_(a, b)
ra = tiedrank_(a); rb = tiedrank_(b); c = corrcoef(ra, rb); r = c(1,2);
end
function r = tiedrank_(x)
[~, o] = sort(x(:)); r = zeros(numel(x),1); r(o) = 1:numel(x);
end
