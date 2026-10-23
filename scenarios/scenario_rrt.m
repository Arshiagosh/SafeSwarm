% scenario_rrt.m - RRT path planning demonstration
clear; clc; close all;
addpath(genpath('..'));

env = Environment([0,50], [0,50]);
env.add_circular_obstacle([20, 20], 6);
env.add_circular_obstacle([35, 35], 5);
%env.add_rectangular_obstacle([10, 16], [35, 45]);

planner = RRT(env, 1000, 2.0);  % step=2.0, max_iter=2000
start   = [2, 2];
goal    = [48, 48];
path    = planner.plan(start, goal);

swarm = Swarm();
for i = 1:4
    agent = Agent(i, [2 + rand, 2 + rand], SingleIntegrator(1.5));
    swarm.add_agent(agent);
end

viz = SwarmVisualizer(swarm, env);
hold on;
plot(path(1,:), path(2,:), 'g-', 'LineWidth', 2);
%plot(start(1), start(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
%plot(goal(1), goal(2), 'r*', 'MarkerSize', 15, 'LineWidth', 2);

behavior = PathFollowing(path, 1.5);
engine   = SimEngine(swarm, env, behavior);
engine.visualizer   = viz;
engine.run(80, 0.1);