clear; clc; close all;
addpath(genpath('..'));

N = 8;
dt = 0.05;
t_max = 30;

agents = cell(1, N);
for i = 1:N
    init_state = [randn(2,1)*5; 0; 0];
    agents{i} = Agent(i, init_state, DoubleIntegrator(2.0, 1.5));
end

swarm = Swarm(agents, 50.0, 'metric');
env = Environment([-30, 30], [-30, 30]);
env.add_circular_obstacle([30, 30], 6);
env.add_rectangular_obstacle([10, 18], [40, 45]);


offsets = [-5 -5  0  5  5  0 -5;
            0 -5 -5 -5  0  5  5];

goal = [20; 20];
path = [agents{1}.state(1:2), goal];
leader_behavior = PathFollowing(path, 5.0);

behav = LeaderFollower(1, offsets, leader_behavior);

viz = SwarmVisualizer(swarm, env);
hold on;
plot(goal(1), goal(2), 'r*', 'MarkerSize', 15, 'LineWidth', 2);

sim = SimEngine(swarm, env, behav, dt, t_max);
sim.visualizer = viz;
sim.run();
