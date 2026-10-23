clear; clc; close all;
addpath(genpath('..'));

N     = 25;
dt    = 0.05;

% phase 1: disperse for 15s, then flock for 20s
phases = {
    struct('behaviour', Dispersion(1.5, 6.0), 't', 15), ...
    struct('behaviour', Flocking(2.0, 1.0, 1.0, 3.0),  't', 20)
};

agents = cell(1, N);
for i = 1:N
    init_state = [randn(2,1)*2; 0; 0];
    agents{i}  = Agent(i, init_state, DoubleIntegrator(3.0, 2.0));
end

swarm = Swarm(agents, 12.0, 'metric');
env   = Environment([-30, 30], [-30, 30]);
viz   = SwarmVisualizer(swarm, env);

t_global = 0;
for p = 1:length(phases)
    phase   = phases{p};
    sim     = SimEngine(swarm, env, phase.behaviour, dt, phase.t);
    sim.visualizer = viz;
    sim.t   = t_global;
    sim.run();
    t_global = t_global + phase.t;
    fprintf('Phase %d done: %s\n', p, class(phase.behaviour));
end
