function varargout = nn_lib(op, varargin)
%NN_LIB  Minimal MLP (ReLU hidden layers, linear output) with Adam, no toolboxes.
%   net = nn_lib('init', sizes)            sizes = [n_in h1 h2 ... n_out], He init (uses global RNG)
%   [Q, cache] = nn_lib('fwd', net, X)     X: n_in x B
%   g = nn_lib('bwd', net, cache, dQ)      gradients of sum(loss) wrt parameters
%   [net, opt] = nn_lib('adam', net, opt, g, lr)
%   net = nn_lib('soft', net_t, net, tau)  Polyak averaging of target network
switch op
    case 'init'
        sz = varargin{1}; L = numel(sz) - 1;
        net.W = cell(L,1); net.b = cell(L,1);
        for l = 1:L
            net.W{l} = randn(sz(l+1), sz(l)) * sqrt(2/sz(l));
            net.b{l} = zeros(sz(l+1), 1);
        end
        varargout{1} = net;
    case 'fwd'
        [net, X] = varargin{:}; L = numel(net.W);
        A = cell(L+1,1); Z = cell(L,1); A{1} = X;
        for l = 1:L
            Z{l} = net.W{l}*A{l} + net.b{l};
            if l < L, A{l+1} = max(0, Z{l}); else, A{l+1} = Z{l}; end
        end
        varargout{1} = A{end}; varargout{2} = struct('A', {A}, 'Z', {Z});
    case 'bwd'
        [net, cache, dQ] = varargin{:}; L = numel(net.W);
        g.W = cell(L,1); g.b = cell(L,1); delta = dQ;
        for l = L:-1:1
            g.W{l} = delta * cache.A{l}';
            g.b{l} = sum(delta, 2);
            if l > 1, delta = (net.W{l}' * delta) .* (cache.Z{l-1} > 0); end
        end
        varargout{1} = g;
    case 'adam'
        [net, opt, g, lr] = varargin{:}; L = numel(net.W);
        b1 = 0.9; b2 = 0.999; ep = 1e-8;
        if isempty(opt)
            opt.t = 0;
            for l = 1:L
                opt.mW{l} = 0*net.W{l}; opt.vW{l} = 0*net.W{l};
                opt.mb{l} = 0*net.b{l}; opt.vb{l} = 0*net.b{l};
            end
        end
        opt.t = opt.t + 1; c1 = 1 - b1^opt.t; c2 = 1 - b2^opt.t;
        for l = 1:L
            opt.mW{l} = b1*opt.mW{l} + (1-b1)*g.W{l};  opt.vW{l} = b2*opt.vW{l} + (1-b2)*g.W{l}.^2;
            opt.mb{l} = b1*opt.mb{l} + (1-b1)*g.b{l};  opt.vb{l} = b2*opt.vb{l} + (1-b2)*g.b{l}.^2;
            net.W{l} = net.W{l} - lr*(opt.mW{l}/c1) ./ (sqrt(opt.vW{l}/c2) + ep);
            net.b{l} = net.b{l} - lr*(opt.mb{l}/c1) ./ (sqrt(opt.vb{l}/c2) + ep);
        end
        varargout{1} = net; varargout{2} = opt;
    case 'soft'
        [nt, net, tau] = varargin{:};
        for l = 1:numel(net.W)
            nt.W{l} = (1-tau)*nt.W{l} + tau*net.W{l};
            nt.b{l} = (1-tau)*nt.b{l} + tau*net.b{l};
        end
        varargout{1} = nt;
    otherwise
        error('nn_lib:op', 'Unknown op %s', op);
end
end
