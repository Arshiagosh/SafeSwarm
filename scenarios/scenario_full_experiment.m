%% scenario_full_experiment — full behaviour stack with analysis
%
% A 12-agent leader–follower formation follows an RRT-planned path from the
% lower-left to the upper-right of a cluttered environment, with inter-agent
% collision avoidance layered on top. Metrics are logged and a full analysis
% dashboard + publication figures are produced at the end.
%
% Behaviour stack: CollisionAvoidance( LeaderFollower( PathFollowing(RRT) ) )

clear; clc; close all;
addpath(genpath('..'));
rng(1);   % reproducible RRT and agent initialisation

%% Setup
N  = 12;
dt = 0.05;

env = Environment([-30, 30], [-30, 30]);
env.add_circular_obstacle([-5, -5], 4);
env.add_circular_obstacle([ 8,  5], 4);
env.add_rectangular_obstacle([-2, 4], [-20, -8]);

agents = cell(1, N);
for i = 1:N
    init_state = [randn(2,1)*3 - 20; 0; 0];   % clustered near [-20, -20]
    agents{i}  = Agent(i, init_state, DoubleIntegrator(2.0, 1.5));
end
swarm = Swarm(agents, 12.0, 'metric');

%% Plan the leader's path with RRT
start_pos = [-20; -20];
goal_pos  = [ 20;  20];
planner   = RRT(env, 8000, 2.0);
[path, ~] = planner.plan(start_pos, goal_pos, 2.5);

if isempty(path)
    error('scenario_full_experiment:NoPath', ...
          'RRT failed to find a path — adjust obstacles, start, or goal.');
end
fprintf('RRT path: %d waypoints\n', size(path, 2));

%% Build the behaviour stack
offsets = 4 * [cos(linspace(pi, 2*pi, N-1));
               sin(linspace(pi, 2*pi, N-1))];

leader_path_ctrl = PathFollowing(path, 4.0);
formation_ctrl   = LeaderFollower(1, offsets, leader_path_ctrl);
full_ctrl        = CollisionAvoidance(formation_ctrl, 2.0, 40.0);

%% Logger + visualizer
logger = DataLogger();
viz    = SwarmVisualizer(swarm, env);
hold on;
plot(path(1,:), path(2,:), 'g--', 'LineWidth', 1.5);

%% Run
sim = SimEngine(swarm, env, full_ctrl, dt, 40);
sim.visualizer = viz;
sim.logger     = logger;
sim.run();

%% Analysis  (results dir resolved relative to the project, not the cwd)
results_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'results');
if ~exist(results_dir, 'dir'); mkdir(results_dir); end

MetricsAnalyzer.print_summary(logger);
logger.export_csv(fullfile(results_dir, 'full_experiment.csv'));

logger.plot_analysis('Full Experiment — Swarm Analysis');
PublicationPlot.trajectory_plot(logger, env, 'Leader-Follower Formation with RRT Path Planning');
PublicationPlot.save_fig(fullfile(results_dir, 'full_experiment_trajectories'));
