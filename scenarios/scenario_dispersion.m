clear; clc; close all;
addpath(genpath('..'));

N     = 25;
dt    = 0.05;
t_max = 30;

agents = cell(1, N);
for i = 1:N
    % start clustered near origin
    init_state = [randn(2,1)*2; 0; 0];
    agents{i}  = Agent(i, init_state, DoubleIntegrator(2.0, 1.5));
end

swarm  = Swarm(agents, 50.0, 'metric');
env    = Environment([-30, 30], [-30, 30]);
behav  = Dispersion(1.5, 6.0);

viz    = SwarmVisualizer(swarm, env);
sim    = SimEngine(swarm, env, behav, dt, t_max);
sim.visualizer = viz;

sim.run();
