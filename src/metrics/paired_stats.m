function S = paired_stats(a, b, block, nboot, seed)
%PAIRED_STATS  Paired comparison of two daily KPI series (e.g. cost of A vs B).
%   Returns the annual difference sum(a-b), a moving-block bootstrap 95 % CI
%   (block length in days, preserves weather autocorrelation) and a two-sided
%   Wilcoxon signed-rank p-value (normal approximation with tie correction).
if nargin < 3, block = 7; end
if nargin < 4, nboot = 2000; end
if nargin < 5, seed = 1; end
d = a(:) - b(:); n = numel(d);
S.diff = sum(d);
rs = rng(); rng(seed, 'twister');
nb = ceil(n/block); bs = zeros(nboot,1);
for i = 1:nboot
    st = randi(n - block + 1, nb, 1);
    idx = bsxfun(@plus, st, 0:block-1)'; idx = idx(1:n);
    bs(i) = sum(d(idx));
end
rng(rs);
bs = sort(bs);
S.ci = [bs(max(1,round(0.025*nboot))), bs(round(0.975*nboot))];
% Wilcoxon signed-rank
dz = d(d ~= 0); m = numel(dz);
if m == 0, S.p = 1; return; end
[~, ord] = sort(abs(dz)); r = zeros(m,1); r(ord) = 1:m;
ad = abs(dz); u = unique(ad); tie = 0;
for i = 1:numel(u)                                 % average ranks for ties
    t = find(ad == u(i)); if numel(t) > 1, r(t) = mean(r(t)); tie = tie + numel(t)^3 - numel(t); end
end
W = sum(r(dz > 0));
mu = m*(m+1)/4; sg = sqrt(m*(m+1)*(2*m+1)/24 - tie/48);
z = (W - mu - 0.5*sign(W - mu)) / sg;
S.p = erfc(abs(z)/sqrt(2));
S.z = z;
end
