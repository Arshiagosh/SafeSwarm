function safeswarm_bench(level)
    % safeswarm_bench(level) — run the tests and regenerate results.json and the figures
    %
    %   safeswarm_bench('quick')  unit tests, E1 benchmarks and the swap GIF (a few minutes)
    %   safeswarm_bench('full')   everything: E1, E2 sampled-data sweep, E3 Monte Carlo
    %                             sweeps, E4 MPC and GP experiments, and the ROS 2 demo
    %                             when the ROS Toolbox is installed (several hours)
    %
    % Every experiment writes its own key in results.json and its figures in figures/.
    % Stops at the first failing test, so the numbers always come from passing code.

    if nargin < 1, level = 'quick'; end
    level = validatestring(level, {'quick', 'full'});
    root  = fileparts(mfilename('fullpath'));
    addpath(genpath(root));

    t0 = tic;
    fprintf('== SafeSwarm %s benchmark ==\n', level);
    run(fullfile(root, 'tests', 'run_tests.m'));

    steps = {'e1_benchmarks'};
    if strcmp(level, 'full')
        steps = [steps, {'e2_sampled_sweep', 'e3a_noise', 'e3b_comm', 'e3c_faults', ...
                         'e4a_mpc_vs_cbf', 'e4b_gp_residual'}];
    end
    for k = 1:numel(steps)
        fprintf('\n== %s ==\n', steps{k});
        run_isolated(fullfile(root, 'experiments', [steps{k} '.m']));
    end
    if strcmp(level, 'full')
        if exist('ros2node', 'file') == 2
            fprintf('\n== demo_ros2_swap ==\n');
            run_isolated(fullfile(root, 'ros2', 'demo_ros2_swap.m'));
        else
            fprintf('\nROS Toolbox not found: skipping the ROS 2 demo.\n');
        end
    end
    fprintf('\nDone in %.1f min. Results: %s\n', toc(t0)/60, fullfile(root, 'results.json'));
end

function run_isolated(script)
    % Each experiment clears its own workspace, so give it a scope of its own.
    run(script);
end
