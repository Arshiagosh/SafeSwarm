classdef MonteCarloRunner
    %% MonteCarloRunner — repeat a seeded experiment and summarise it
    %
    % fn(seed) must run one simulation and return a struct of scalar
    % metrics (for example the output of DataLogger.summary or of the
    % experiments' run_case). Every run gets its own seed, set with rng
    % before fn is called, so the whole batch is reproducible.
    %
    % Example:
    %   rows = MonteCarloRunner.run(@(seed) my_case(seed), 1:20);
    %   s    = MonteCarloRunner.summarise(rows, 'min_distance');
    %   rate = MonteCarloRunner.rate(rows, @(r) r.agent_collisions > 0);

    methods (Static)
        function rows = run(fn, seeds, verbose)
            % run(fn, seeds, verbose) — struct array with one row per seed
            if nargin < 3, verbose = false; end
            rows = cell(1, numel(seeds));
            for k = 1:numel(seeds)
                rng(seeds(k));
                r = fn(seeds(k));
                r.seed = seeds(k);
                rows{k} = r;
                if verbose
                    fprintf('  seed %d done (%d/%d)\n', seeds(k), k, numel(seeds));
                end
            end
            rows = [rows{:}];
        end

        function s = summarise(rows, field)
            % summarise(rows, field) — mean, std, min, max and median of one field
            x = [rows.(field)];
            x = x(~isnan(x));
            s = struct('mean', mean(x), 'std', std(x), 'min', min(x), ...
                       'max', max(x), 'median', median(x), 'n', numel(x));
        end

        function [p, lo, hi] = rate(rows, pred)
            % rate(rows, pred) — fraction of runs where pred(row) is true,
            %   with a 95 % Wilson score interval [lo, hi].
            k = sum(arrayfun(pred, rows));
            n = numel(rows);
            p = k / n;
            z = 1.96;
            c = (p + z^2/(2*n)) / (1 + z^2/n);
            w = z * sqrt(p*(1-p)/n + z^2/(4*n^2)) / (1 + z^2/n);
            lo = max(0, c - w);
            hi = min(1, c + w);
        end
    end
end
