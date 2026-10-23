%% scenario_potential_field — Potential Field path planning demo
%
% Demonstrates the PotentialField planner navigating 10 agents from a
% scattered start cluster to a shared goal in a large environment with
% two circular and one rectangular obstacle.
%
% Dynamics:  DoubleIntegrator
% Behaviour: PathFollowing (path computed by PotentialField)
% Agents:    10

clear; clc; close all;
addpath(genpath('..'));

%% Environment
env = Environment([-100, 100], [-100, 100]);
env.add_circular_obstacle([-10, 0], 10);
env.add_circular_obstacle([15, 15], 4);
env.add_rectangular_obstacle([30, 35], [10, 22]);

%% Plan a shared path for the swarm
planner = PotentialField(env, 0.5, 500, 5.0);

start_pos = [-20; -20];
goal_pos  = [20;  20];

tic;
path = planner.plan(start_pos, goal_pos, 50000, 0.1);
t_plan = toc;

fprintf('Potential Field planning time: %.3f s\n', t_plan);
fprintf('Path length: %d waypoints\n', size(path, 2));

%% Build swarm — agents start near start_pos with bounded random offsets
N = 10;
agents = cell(1, N);
rng(42);  % reproducible initialization
for i = 1:N
    % Uniform offset within ±15 m keeps all agents well inside [-100,100]
    offset = (rand(2,1) - 0.5) * 30;
    init_state = [start_pos + offset; 0; 0];
    agents{i} = Agent(i, init_state, DoubleIntegrator(2.0, 1.5));
end

swarm = Swarm(agents, 10.0, 'metric');
behav = PathFollowing(path, 3.0);

%% Visualize
viz = SwarmVisualizer(swarm, env);
hold on;
plot(path(1,:), path(2,:), 'g-',  'LineWidth', 2);
plot(start_pos(1), start_pos(2),  'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(goal_pos(1),  goal_pos(2),   'r*', 'MarkerSize', 15, 'LineWidth', 2);

%% Run simulation
sim = SimEngine(swarm, env, behav, 0.05, 400);
sim.visualizer = viz;
sim.run();
