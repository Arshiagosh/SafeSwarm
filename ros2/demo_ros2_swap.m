% demo_ros2_swap — the SafeSwarm filter as a ROS 2 node between controller and robots
%
% Three ROS 2 nodes talk only over topics:
%   /sim_robots        8 simulated robots: publish <ns>/odom, apply <ns>/cmd_safe
%   /nominal           go-to-goal controller: odom in, <ns>/cmd_nominal out
%   /safeswarm_filter  centralised CBF-QP filter: odom + cmd_nominal in, cmd_safe out
% Each 0.05 s cycle: robots publish odometry -> controller -> filter -> robots.
% Runs the antipodal swap twice: once with the robots listening to
% cmd_nominal directly (no filter in the loop) and once to cmd_safe.
% Prints the closest approach of both runs, and writes figures/ros2_swap.png
% and the "ros2" key of results.json.
%
% Needs the ROS Toolbox (ROS 2). All nodes run in this MATLAB session on the
% local machine; on real robots the filter node would run the same way and
% <ns>/cmd_safe would be remapped to each robot's velocity topic.

clear; close all;
root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(root));
reset(groot);

N = 8; dt = 0.05; t_max = 20;
names = arrayfun(@(i) sprintf('/robot_%d', i), 1:N, 'UniformOutput', false);
results = struct('name', {}, 'min_distance', {}, 'agent_collisions', {}, 'late', {});
logs = {};

for use_filter = [false true]
    rng(1);
    ang = 2*pi*(0:N-1)/N;
    P0  = 5 * [cos(ang); sin(ang)] + 0.05 * randn(2, N);
    G   = -P0;
    mk  = @() make_swarm(P0);
    env = Environment([-7 7], [-7 7]);

    if use_filter, robot_topic = 'cmd_safe'; else, robot_topic = 'cmd_nominal'; end
    sim     = SimRobotsNode('/sim_robots', names, mk(), robot_topic, env);
    nominal = BehaviourNode('/nominal', names, mk(), GoToGoal(G, 1.0), 'cmd_nominal', env);
    if use_filter
        fnode = ros2node('/safeswarm_filter_in');
        cmds  = TopicCommand(fnode, names, mk(), 'cmd_nominal');
        filt  = BehaviourNode('/safeswarm_filter', names, mk(), ...
                              SafetyFilter(cmds, 'Mode', 'centralised'), 'cmd_safe', env);
    end
    pause(2);   % let discovery finish

    for k = 1:round(t_max / dt)
        t = sim.t;
        sim.publish_odom();
        nominal.step(t);
        if use_filter, filt.step(t); end
        sim.step(dt);
    end

    s = sim.logger.summary();
    late = sim.n_late + nominal.n_late;
    if use_filter, late = late + filt.n_late + cmds.n_late; end
    results(end+1) = struct('name', robot_topic, 'min_distance', s.min_distance, ...
                            'agent_collisions', s.agent_collisions, 'late', late); %#ok<SAGROW>
    fprintf('robots on %-11s: closest approach %.3f m, collision steps %d, late cycles %d\n', ...
            robot_topic, s.min_distance, s.agent_collisions, late);
    logs{end+1} = sim.logger; %#ok<SAGROW>
    clear sim nominal filt cmds fnode
end

f = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 900 420]);
tl = tiledlayout(f, 1, 2, 'TileSpacing', 'compact');
for k = 1:2
    ax = nexttile(tl); hold(ax, 'on'); axis(ax, 'equal'); box(ax, 'on');
    P = logs{k}.positions();
    for i = 1:N
        c = hsv2rgb([(i-1)/N, 0.75, 0.9]);
        plot(ax, squeeze(P(1, i, :)), squeeze(P(2, i, :)), 'Color', c, 'LineWidth', 1.2);
        plot(ax, P(1, i, 1), P(2, i, 1), 'o', 'Color', c, 'MarkerSize', 4);
    end
    axis(ax, [-6.5 6.5 -6.5 6.5]);
    title(ax, sprintf('robots on %s: d_{min} %.2f m', strrep(results(k).name, '_', '\_'), ...
                      results(k).min_distance), 'FontWeight', 'normal');
end
title(tl, 'Antipodal swap over ROS 2 topics');
print(f, fullfile(root, 'figures', 'ros2_swap.png'), '-dpng', '-r150');
close(f);

file = fullfile(root, 'results.json');
data = jsondecode(fileread(file));
data.ros2 = results;
fid = fopen(file, 'w');
fprintf(fid, '%s\n', jsonencode(data, 'PrettyPrint', true));
fclose(fid);

function swarm = make_swarm(P0)
    N = size(P0, 2);
    agents = cell(1, N);
    for i = 1:N
        agents{i} = Agent(i, P0(:, i), SingleIntegrator(1.0));
    end
    swarm = Swarm(agents, 10, 'full');
    swarm.agent_radius = 0.3;
end
