classdef MetricsAnalyzer < handle
    %% MetricsAnalyzer — post-simulation performance analysis
    %
    % All methods are static; pass a DataLogger populated by a completed
    % simulation. Results can be printed or used for comparison plots.
    %
    % Example:
    %   MetricsAnalyzer.print_summary(logger);
    %   t_c = MetricsAnalyzer.convergence_time(logger, 3.0);

    methods (Static)
        function t_conv = convergence_time(logger, threshold)
            % convergence_time(logger, threshold) — time when spread < threshold permanently
            %   threshold : spread threshold in metres (default 3.0)
            %   Returns NaN if convergence is never reached.
            if nargin < 2, threshold = 3.0; end
            spread = [logger.metric_log.spread];
            t      = logger.time_log;
            t_conv = NaN;
            for k = 1:length(spread)
                if all(spread(k:end) < threshold)
                    t_conv = t(k);
                    return;
                end
            end
        end

        function e_total = total_energy(logger)
            % total_energy(logger) — time-integrated kinetic energy (J·s)
            e_total = trapz(logger.time_log, [logger.metric_log.energy]);
        end

        function len = path_length(logger, agent_idx)
            % path_length(logger, agent_idx) — total distance travelled by one agent (m)
            %   agent_idx : column index into state matrices (default 1)
            if nargin < 2, agent_idx = 1; end
            len = 0;
            for k = 2:length(logger.state_log)
                p_prev = logger.state_log{k-1}(1:2, agent_idx);
                p_curr = logger.state_log{k  }(1:2, agent_idx);
                len    = len + norm(p_curr - p_prev);
            end
        end

        function r = connectivity_ratio(logger)
            % connectivity_ratio(logger) — fraction of time the graph was connected
            %   A graph is connected when its Fiedler eigenvalue (λ₂) > 0.
            conn = [logger.metric_log.connectivity];
            r    = mean(conn > 1e-6);
        end

        function print_summary(logger)
            % print_summary(logger) — print all metrics to the console
            fprintf('\n===== Simulation Metrics Summary =====\n');
            fprintf('Duration             : %.2f s\n', logger.time_log(end));
            fprintf('Steps logged         : %d\n',     logger.log_count);
            fprintf('Convergence time     : %.2f s\n', MetricsAnalyzer.convergence_time(logger));
            fprintf('Total energy         : %.2f J\n', MetricsAnalyzer.total_energy(logger));
            fprintf('Connectivity ratio   : %.1f%%\n', MetricsAnalyzer.connectivity_ratio(logger)*100);
            fprintf('Path length (agent 1): %.2f m\n', MetricsAnalyzer.path_length(logger, 1));
            fprintf('======================================\n');
        end
    end
end
