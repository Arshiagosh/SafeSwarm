classdef ExperimentRunner < handle
    %% ExperimentRunner — batch harness for running and comparing scenarios
    %
    % Register named scenario functions, run them all, export per-scenario
    % CSVs, and produce a comparison bar chart across convergence time,
    % energy, and connectivity.
    %
    % Each scenario_fn must return a populated DataLogger:
    %   fn = @() my_scenario();   % function that runs sim and returns logger
    %
    % Example:
    %   runner = ExperimentRunner();
    %   runner.add('Flocking',    @() run_flocking());
    %   runner.add('Aggregation', @() run_aggregation());
    %   runner.run_all('results/');

    properties
        experiments = {};   % cell array of struct(name, fn)
        results     = {};   % cell array of struct(name, logger) after run_all
    end

    methods
        function add(obj, name, scenario_fn)
            % add(name, scenario_fn) — register a scenario
            %   name        : string label used in plots and CSV filenames
            %   scenario_fn : function handle @() → DataLogger
            obj.experiments{end+1} = struct('name', name, 'fn', scenario_fn);
        end

        function run_all(obj, output_dir)
            % run_all(output_dir) — execute all registered scenarios
            %   output_dir : directory for CSV outputs (default 'results')
            if nargin < 2, output_dir = 'results'; end
            if ~exist(output_dir, 'dir'), mkdir(output_dir); end

            for k = 1:length(obj.experiments)
                exp = obj.experiments{k};
                fprintf('\n[%d/%d] Running: %s\n', k, length(obj.experiments), exp.name);

                tic;
                logger  = exp.fn();
                elapsed = toc;

                fprintf('  Done in %.2f s\n', elapsed);
                MetricsAnalyzer.print_summary(logger);
                logger.export_csv(fullfile(output_dir, [exp.name '.csv']));
                obj.results{end+1} = struct('name', exp.name, 'logger', logger);
            end

            obj.plot_comparison();
        end

        function plot_comparison(obj)
            % plot_comparison() — bar chart comparing all registered experiments
            if isempty(obj.results), return; end

            names     = cellfun(@(r) r.name,                                   obj.results, 'UniformOutput', false);
            t_convs   = cellfun(@(r) MetricsAnalyzer.convergence_time(r.logger),   obj.results);
            energies  = cellfun(@(r) MetricsAnalyzer.total_energy(r.logger),        obj.results);
            conn_rats = cellfun(@(r) MetricsAnalyzer.connectivity_ratio(r.logger),  obj.results);

            figure('Name', 'Experiment Comparison', 'NumberTitle', 'off');

            subplot(1,3,1);
            bar(t_convs);
            set(gca, 'XTickLabel', names, 'XTickLabelRotation', 30);
            ylabel('Convergence Time (s)'); title('Convergence'); grid on;

            subplot(1,3,2);
            bar(energies);
            set(gca, 'XTickLabel', names, 'XTickLabelRotation', 30);
            ylabel('Total Energy (J)'); title('Energy'); grid on;

            subplot(1,3,3);
            bar(conn_rats * 100);
            set(gca, 'XTickLabel', names, 'XTickLabelRotation', 30);
            ylabel('Connectivity (%)'); title('Connectivity'); grid on;
        end
    end
end
