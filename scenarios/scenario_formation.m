% scenario_formation — displacement-based formation of six double integrators
%
% Six agents start at random positions and form a hexagon of radius 4 m
% using only relative positions and velocities over a ring graph, while the
% formation moves at 0.3 m/s. Writes figures/formation.gif and
% figures/formation_error.png.

clear; clc; close all;
root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(root));
rng(7);

N     = 6;
dt    = 0.05;
t_max = 25;

agents = cell(1, N);
for i = 1:N
    agents{i} = Agent(i, [randn(2,1)*4; 0; 0], DoubleIntegrator(2.0, 1.5));
end
swarm = Swarm(agents, 50.0, 'metric');
swarm.agent_radius = 0.3;
env   = Environment([-10, 15], [-10, 15]);

% Hexagon shape and a ring graph (each agent talks to two neighbours)
ang   = 2*pi*(0:N-1)/N;
shape = 4 * [cos(ang); sin(ang)];
ring  = circshift(eye(N), 1) + circshift(eye(N), -1);

behav       = DisplacementFormation(shape, ring, 1.0);
behav.v_ref = [0.3; 0.3];

fig_dir = fullfile(root, 'figures');
if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end

viz = SwarmVisualizer(swarm, env, 'Theme', 'light', 'Visible', 'off', ...
                      'GifFile', fullfile(fig_dir, 'formation.gif'), 'GifEvery', 5, ...
                      'Title', 'Displacement formation');
sim = SimEngine(swarm, env, behav, dt, t_max);
sim.contact_mode = 'none';
sim.visualizer   = viz;
sim.logger       = DataLogger();

% Track the formation error alongside the run
n_steps = round(t_max / dt);
err     = zeros(1, n_steps + 1);
err(1)  = behav.formation_error(swarm);
sim.logger.log(0, swarm, env);
for k = 1:n_steps
    sim.step();
    err(k + 1) = behav.formation_error(swarm);
end
viz.capture_frame(true);
viz.close();

f = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 620 360]);
semilogy((0:n_steps) * dt, err, 'LineWidth', 1.6); grid on;
xlabel('Time (s)', 'Interpreter', 'none');
ylabel('Mean formation error (m)', 'Interpreter', 'none');
title('Displacement-based formation, ring graph, N = 6', 'Interpreter', 'none');
print(f, fullfile(fig_dir, 'formation_error.png'), '-dpng', '-r150');
close(f);

s = sim.logger.summary();
fprintf('Formation error: %.3f m at t = 0, %.1e m at t = %.0f s\n', err(1), err(end), t_max);
fprintf('Closest approach: %.2f m (agent diameter %.1f m)\n', s.min_distance, 2*swarm.agent_radius);
