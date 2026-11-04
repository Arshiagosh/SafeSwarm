classdef TopicCommand < handle
    %% TopicCommand — a behaviour whose commands come from ROS 2 topics
    %
    % Subscribes to <ns>/<topic> for every robot and returns the latest
    % commands as a control_dim×N matrix. Wrap it in a SafetyFilter to filter
    % commands produced by other nodes (any language, any controller).
    % compute_control waits (up to timeout) for commands stamped with the
    % swarm's current time; late ones are counted in n_late and the newest
    % received command is used (zero before the first one arrives).
    %
    % Requires the ROS Toolbox.

    properties
        subs = {};
        dyns = {};
        timeout = 1.0;
        n_late  = 0;
    end

    methods
        function obj = TopicCommand(node, names, swarm, topic)
            % TopicCommand(node, names, swarm, topic)
            %   node  : ros2node that owns the subscriptions
            %   names : 1×N cell of robot namespaces
            %   swarm : Swarm (for each robot's dynamics, hence message type)
            %   topic : e.g. 'cmd_nominal'
            for i = 1:numel(names)
                obj.dyns{i} = swarm.agents{i}.dynamics;
                obj.subs{i} = ros2subscriber(node, [names{i} '/' topic], ...
                                             RosTopics.cmd_type(obj.dyns{i}), ...
                                             'Reliability', 'reliable', 'Depth', 5);
            end
        end

        function u = compute_control(obj, swarm, ~)
            % compute_control(swarm, env) — latest received commands
            n = numel(obj.subs);
            got = false(1, n);
            t0 = tic;
            while true
                for i = find(~got)
                    m = obj.subs{i}.LatestMessage;
                    if ~isempty(m) && abs(RosTopics.seconds(m.header.stamp) - swarm.time) < 1e-6
                        got(i) = true;
                    end
                end
                if all(got) || toc(t0) > obj.timeout, break; end
                pause(0.001);
            end
            obj.n_late = obj.n_late + ~all(got);
            u = zeros(2, n);
            for i = 1:n
                m = obj.subs{i}.LatestMessage;
                if ~isempty(m), u(:, i) = RosTopics.read_cmd(m, obj.dyns{i}); end
            end
        end
    end
end
