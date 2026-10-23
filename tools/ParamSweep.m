classdef ParamSweep < handle
    %% ParamSweep — parametric sweep over a single scenario parameter
    %
    % Calls scenario_fn for each value of a parameter and computes a scalar
    % metric from the resulting DataLogger. Results are stored and can be
    % plotted. Useful for tuning gains, communication range, swarm size, etc.
    %
    % Example:
    %   ps = ParamSweep();
    %   ps.sweep('comm_radius', [5 10 15 20 25], ...
    %       @(r) run_flocking_with_radius(r), ...
    %       @(log) MetricsAnalyzer.convergence_time(log));
    %   ps.plot_sweep('comm_radius', 'Convergence Time (s)');

    properties
        results = struct();   % stores sweep results keyed by param_name
    end

    methods
        function sweep(obj, param_name, param_values, scenario_fn, metric_fn)
            % sweep(param_name, param_values, scenario_fn, metric_fn)
            %   param_name   : string label for the swept parameter
            %   param_values : vector of values to test
            %   scenario_fn  : @(param_val) → DataLogger
            %   metric_fn    : @(DataLogger) → scalar
            n       = length(param_values);
            metrics = zeros(1, n);
            for k = 1:n
                fprintf('Running %s = %.4g  (%d/%d)\n', param_name, param_values(k), k, n);
                logger     = scenario_fn(param_values(k));
                metrics(k) = metric_fn(logger);
            end
            obj.results.(param_name) = struct('values', param_values, 'metrics', metrics);
        end

        function plot_sweep(obj, param_name, y_label)
            % plot_sweep(param_name, y_label) — plot metric vs. parameter value
            %   param_name : must match a name previously passed to sweep()
            %   y_label    : y-axis label string
            r = obj.results.(param_name);
            figure('Name', ['Sweep: ' param_name], 'NumberTitle', 'off');
            plot(r.values, r.metrics, 'bo-', 'LineWidth', 1.5, 'MarkerSize', 6);
            xlabel(param_name); ylabel(y_label); grid on;
            title(['Parameter Sweep: ' param_name]);
        end
    end
end
