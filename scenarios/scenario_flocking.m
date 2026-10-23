clear; clc; close all;
addpath(genpath('..'));

%% parameters
N        = 30;
dt       = 0.05;
t_max    = 30;
x_lim    = [-30, 30];
y_lim    = [-30, 30];

%% build agents (double integrator)
agents = cell(1, N);
for i = 1:N
    init_state = [rand(2,1)*20 - 10; rand(2,1)*2 - 1];  % random pos + vel
    dyn        = DoubleIntegrator(3.0, 2.0);
    agents{i}  = Agent(i, init_state, dyn);
end

%% build swarm, environment, behaviour
swarm  = Swarm(agents, 10.0, 'metric');
env    = Environment(x_lim, y_lim);
behav  = Flocking(2.0, 1.0, 1.0, 3.0);

%% visualizer and engine
viz    = SwarmVisualizer(swarm, env);
sim    = SimEngine(swarm, env, behav, dt, t_max);
sim.visualizer = viz;

%% run
sim.run();
