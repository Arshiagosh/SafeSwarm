clear; clc; close all;
addpath(genpath('..'));

N = 7;
dt = 0.05;
t_max = 100;

agents = cell(1, N);
for i = 1:N
    init_state = [randn(2,1)*3; 0; 0];
    agents{i} = Agent(i, init_state, DoubleIntegrator(2.0, 1.5));
end

swarm = Swarm(agents, 50.0, 'metric');
env = Environment([-30, 30], [-30, 30]);
%env.add_circular_obstacle([20, 20], 5);
%env.add_circular_obstacle([-20, -20], 5);

% V-shape formation
angles = linspace(pi*0.75, pi*0.25, N);
r = 6.0;
formation = r * [cos(angles); sin(angles)];

virtual_center = [0; 0];
vs = VirtualStructure(formation, virtual_center);
vs.set_virtual_velocity([0.5; 0.5]);

behav = FormationWithObstacles(vs, 80.0, 4.0);

viz = SwarmVisualizer(swarm, env);
sim = SimEngine(swarm, env, behav, dt, t_max);
sim.visualizer = viz;
sim.run();
