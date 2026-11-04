classdef SimRobotsNode < handle
    %% SimRobotsNode — simulated robots that talk ROS 2
    %
    % Stands in for real robots in the ROS 2 demo: publishes <ns>/odom for
    % every robot and integrates the latest command received on
    % <ns>/<cmd_topic> with the robot's SafeSwarm dynamics. A DataLogger
    % records the true states, so collisions are measured on the ground truth.
    %
    % Requires the ROS Toolbox.

    properties
        node
        names
        swarm           % ground-truth Swarm
        env = [];
        logger
        t = 0;
        timeout = 1.0;
        n_late  = 0;    % steps in which some command was not fresh
        subs = {};
        pubs = {};
        msgs = {};
    end

    methods
        function obj = SimRobotsNode(node_name, names, swarm, cmd_topic, env)
            if nargin > 4, obj.env = env; end
            obj.node  = ros2node(node_name);
            obj.names = names;
            obj.swarm = swarm;
            obj.logger = DataLogger();
            for i = 1:numel(names)
                dyn = swarm.agents{i}.dynamics;
                obj.pubs{i} = ros2publisher(obj.node, [names{i} '/odom'], 'nav_msgs/Odometry', ...
                                            'Reliability', 'reliable');
                obj.msgs{i} = ros2message(obj.pubs{i});
                obj.subs{i} = ros2subscriber(obj.node, [names{i} '/' cmd_topic], ...
                                             RosTopics.cmd_type(dyn), ...
                                             'Reliability', 'reliable', 'Depth', 5);
            end
        end

        function publish_odom(obj)
            % Send every robot's current state, stamped with the current time.
            for i = 1:obj.swarm.N
                obj.msgs{i} = RosTopics.fill_odom(obj.msgs{i}, obj.swarm.agents{i}.state, obj.t);
                send(obj.pubs{i}, obj.msgs{i});
            end
        end

        function step(obj, dt)
            % Wait for commands stamped t, apply them for dt seconds, log.
            n = obj.swarm.N;
            got = false(1, n);
            t0 = tic;
            while true
                for i = find(~got)
                    m = obj.subs{i}.LatestMessage;
                    if ~isempty(m) && abs(RosTopics.seconds(m.header.stamp) - obj.t) < 1e-6
                        got(i) = true;
                    end
                end
                if all(got) || toc(t0) > obj.timeout, break; end
                pause(0.001);
            end
            obj.n_late = obj.n_late + ~all(got);
            if obj.logger.log_count == 0
                obj.logger.log(obj.t, obj.swarm, obj.env);
            end
            U = zeros(2, n);
            for i = 1:n
                a = obj.swarm.agents{i};
                m = obj.subs{i}.LatestMessage;
                if ~isempty(m), U(:, i) = RosTopics.read_cmd(m, a.dynamics); end
                a.step(U(:, i), dt);
            end
            obj.t = obj.t + dt;
            obj.swarm.time = obj.t;
            obj.swarm.invalidate_cache();
            obj.logger.log(obj.t, obj.swarm, obj.env, U);
        end
    end
end
