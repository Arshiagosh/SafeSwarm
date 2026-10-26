% scenario_swap_cbf — antipodal swap with and without the CBF safety filter
%
% Eight single-integrator robots on a circle drive to the opposite side.
% Without a filter they meet in the middle; with the decentralised CBF-QP
% filter they pass each other. Prints the closest approach of both runs.

clear; clc; close all;
root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(root));

for use_filter = [false true]
    rng(1);
    N   = 8;
    ang = 2*pi*(0:N-1)/N;
    P0  = 5 * [cos(ang); sin(ang)] + 0.05 * randn(2, N);   % small offsets break symmetry
    agents = cell(1, N);
    for i = 1:N
        agents{i} = Agent(i, P0(:, i), SingleIntegrator(1.0));
    end
    swarm = Swarm(agents, 10, 'full');
    swarm.agent_radius = 0.3;
    env   = Environment([-7 7], [-7 7]);

    behav = GoToGoal(-P0, 1.0);
    if use_filter
        behav = SafetyFilter(behav, 'Mode', 'decentralised');
    end
    sim = SimEngine(swarm, env, behav, 0.02, 15);
    sim.contact_mode = 'none';
    sim.logger = DataLogger();
    sim.run();
    s = sim.logger.summary();
    fprintf('filter = %d: closest approach %.3f m (agent diameter %.2f m), collision steps %d\n', ...
            use_filter, s.min_distance, 2*swarm.agent_radius, s.agent_collisions);
end
