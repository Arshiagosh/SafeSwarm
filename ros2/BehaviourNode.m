classdef BehaviourNode < handle
    %% BehaviourNode — run any SafeSwarm behaviour as a ROS 2 node
    %
    % The node subscribes to <ns>/odom of every robot, keeps a mirror Swarm
    % with the latest received states, runs the behaviour on it and publishes
    % one command per robot on <ns>/<out_topic>. Two typical uses:
    %
    %   nominal = BehaviourNode('/nominal', ns, mirror, GoToGoal(goals), 'cmd_nominal');
    %   cmds    = TopicCommand(node, ns, mirror, 'cmd_nominal');      % read them back
    %   filter  = BehaviourNode('/safeswarm_filter', ns, mirror2, ...
    %                           SafetyFilter(cmds, 'Mode', 'centralised'), 'cmd_safe');
    %
    % step(t) handles one control cycle: it waits (up to timeout) until every
    % robot's odometry is stamped t, runs the behaviour and publishes with
    % stamp t. Late messages are counted in n_late; the node then uses the
    % newest state it has.
    %
    % Requires the ROS Toolbox.

    properties
        node            % ros2node
        names           % 1×N cell of robot namespaces, e.g. {'/robot_1', ...}
        mirror          % Swarm whose states are overwritten from odometry
        env = [];       % Environment passed to the behaviour
        behaviour       % any SafeSwarm behaviour
        timeout = 1.0;  % seconds to wait for fresh odometry
        n_late  = 0;    % cycles in which some odometry was not fresh
        subs = {};
        pubs = {};
        msgs = {};
    end

    methods
        function obj = BehaviourNode(node_name, names, mirror, behaviour, out_topic, env)
            % BehaviourNode(node_name, names, mirror, behaviour, out_topic, env)
            if nargin > 5, obj.env = env; end
            obj.node = ros2node(node_name);
            obj.names = names;
            obj.mirror = mirror;
            obj.behaviour = behaviour;
            for i = 1:numel(names)
                dyn = mirror.agents{i}.dynamics;
                obj.subs{i} = ros2subscriber(obj.node, [names{i} '/odom'], 'nav_msgs/Odometry', ...
                                             'Reliability', 'reliable', 'Depth', 5);
                obj.pubs{i} = ros2publisher(obj.node, [names{i} '/' out_topic], ...
                                            RosTopics.cmd_type(dyn), 'Reliability', 'reliable');
                obj.msgs{i} = ros2message(obj.pubs{i});
            end
        end

        function fresh = update_mirror(obj, t)
            % Copy the latest odometry into the mirror; wait for stamp t.
            n = numel(obj.names);
            got = false(1, n);
            t0 = tic;
            while true
                for i = find(~got)
                    m = obj.subs{i}.LatestMessage;
                    if ~isempty(m) && abs(RosTopics.seconds(m.header.stamp) - t) < 1e-6
                        got(i) = true;
                    end
                end
                if all(got) || toc(t0) > obj.timeout, break; end
                pause(0.001);
            end
            for i = 1:n
                m = obj.subs{i}.LatestMessage;
                if ~isempty(m)
                    a = obj.mirror.agents{i};
                    a.state = RosTopics.read_odom(m, numel(a.state));
                end
            end
            obj.mirror.time = t;
            obj.mirror.invalidate_cache();
            fresh = all(got);
            obj.n_late = obj.n_late + ~fresh;
        end

        function u = step(obj, t)
            % step(t) — one control cycle at time t; returns the published commands
            obj.update_mirror(t);
            u = obj.behaviour.compute_control(obj.mirror, obj.env);
            for i = 1:numel(obj.names)
                dyn = obj.mirror.agents{i}.dynamics;
                obj.msgs{i} = RosTopics.fill_cmd(obj.msgs{i}, dyn, u(:, i), t);
                send(obj.pubs{i}, obj.msgs{i});
            end
        end
    end
end
