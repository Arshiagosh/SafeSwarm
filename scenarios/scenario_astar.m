%% scenario_astar — A* grid path planning demonstration
%
% Plans a path with A* through an environment with circular and rectangular
% obstacles, then has a small swarm follow it. Trajectories are plotted at
% the end with PublicationPlot.
%
% Dynamics:  SingleIntegrator
% Behaviour: PathFollowing (path from AStar)
% Agents:    5

clear; clc; close all;
addpath(genpath('..'));

%% Environment
env = Environment([0, 50], [0, 50]);
env.add_circular_obstacle([15, 25], 4);
env.add_circular_obstacle([30, 15], 5);
env.add_rectangular_obstacle([20, 30], [28, 34]);

%% Plan with A*
planner   = AStar(env, 1.0);          % grid resolution = 1.0 m
start_pos = [2; 2];
goal_pos  = [48; 48];
path = planner.plan(start_pos, goal_pos);

if isempty(path)
    error('scenario_astar:NoPath', 'A* failed to find a path.');
end
fprintf('A* found a path with %d waypoints\n', size(path, 2));

%% Swarm
num_agents = 5;
agents = cell(1, num_agents);
rng(7);
for i = 1:num_agents
    init_pos  = start_pos + randn(2,1) * 0.5;
    agents{i} = Agent(i, init_pos, SingleIntegrator(1.5));
end
swarm = Swarm(agents, 50.0, 'metric');

%% Behaviour + simulation (with logging for analysis)
behav  = PathFollowing(path, 1.5);
logger = DataLogger();
sim    = SimEngine(swarm, env, behav, 0.1, 60);
sim.logger = logger;
sim.run();

%% Results
fprintf('Simulation complete.\n');
PublicationPlot.trajectory_plot(logger, env, 'A* Path Following');
hold on;
plot(path(1,:), path(2,:), 'g--', 'LineWidth', 1.5);
plot(goal_pos(1), goal_pos(2), 'r*', 'MarkerSize', 15, 'LineWidth', 2);
