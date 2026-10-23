clear; clc; close all;
addpath(genpath('..'));

sweeper = ParamSweep();

scenario_fn = @(comm_range) run_scenario(comm_range);
metric_fn   = @(logger) MetricsAnalyzer.convergence_time(logger, 4.0);

sweeper.sweep('comm_range', [5 8 12 18 25 40], scenario_fn, metric_fn);
sweeper.plot_sweep('comm_range', 'Convergence Time (s)');

function logger = run_scenario(comm_range)
    N  = 15;
    dt = 0.05;

    agents = cell(1, N);
    for i = 1:N
        init_state = [randn(2,1)*15; 0; 0];
        agents{i}  = Agent(i, init_state, DoubleIntegrator(2.0, 1.5));
    end

    swarm  = Swarm(agents, comm_range, 'metric');
    env    = Environment([-30, 30], [-30, 30]);
    env.add_circular_obstacle([25, 25], 6);
    env.add_rectangular_obstacle([10, 18], [10, 18]);

    behav  = CollisionAvoidance(Aggregation(1.5), 2.0, 30.0);
    logger = DataLogger();

    sim = SimEngine(swarm, env, behav, dt, 25);
    sim.logger = logger;
    sim.run();
end
