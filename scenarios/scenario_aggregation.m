clear; clc; close all;
addpath(genpath('..'));

N     = 20;
dt    = 0.05;
t_max = 20;

agents = cell(1, N);
for i = 1:N
    init_state = [rand(2,1)*20 - 20; 0; 0];
    agents{i}  = Agent(i, init_state, DoubleIntegrator());
end

swarm  = Swarm(agents, 50.0, 'metric');   % large radius = all see all
env    = Environment([-25, 25], [-25, 25]);
behav  = Aggregation(1.5);

viz    = SwarmVisualizer(swarm, env);
sim    = SimEngine(swarm, env, behav, dt, t_max);
sim.visualizer = viz;

sim.run();
