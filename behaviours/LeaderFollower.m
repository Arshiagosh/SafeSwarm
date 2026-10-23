classdef LeaderFollower < handle
    %% LeaderFollower — hierarchical formation control with one designated leader
    %
    % Agent leader_idx is the leader; all other agents are followers. Followers
    % track fixed offsets from the leader's position using proportional-derivative
    % control on position and velocity errors.
    %
    % The leader can be given its own base behaviour (e.g. PathFollowing) that
    % drives it through the environment while followers maintain formation.
    %
    % Works with: DoubleIntegrator (requires velocity state for vel_error term)
    % Use case:   search & rescue, convoy, hierarchical coordination
    %
    % Example:
    %   offsets = [0 2 -2 0; 2 0 0 -2];   % 2×(N-1) offset matrix
    %   behav   = LeaderFollower(1, offsets);
    %   sim     = SimEngine(swarm, env, behav);

    properties
        leader_idx        = 1;      % index of leader agent in swarm.agents
        formation_offsets           % 2×(N-1) desired offsets from leader (metres)
        k_formation       = 2.0;    % position error gain
        k_velocity        = 1.0;    % velocity matching gain
        base_behavior               % optional behaviour driving the leader
        max_accel         = 3.0;    % control saturation (m/s²)
    end

    methods
        function obj = LeaderFollower(leader_idx, offsets, base_behavior)
            % LeaderFollower(leader_idx, offsets, base_behavior)
            %   leader_idx    : integer index of leader agent (default 1)
            %   offsets       : 2×(N-1) matrix of follower offsets
            %   base_behavior : optional behaviour for the leader (default none)
            obj.leader_idx        = leader_idx;
            obj.formation_offsets = offsets;
            if nargin > 2
                obj.base_behavior = base_behavior;
            end
        end

        function u = compute_control(obj, swarm, env)
            % compute_control(swarm, env) — returns control_dim×N control matrix
            ctrl_dim = swarm.agents{1}.dynamics.control_dim;
            u        = zeros(ctrl_dim, swarm.N);

            leader     = swarm.agents{obj.leader_idx};
            leader_pos = leader.state(1:2);
            leader_vel = leader.state(3:4);

            if ~isempty(obj.base_behavior)
                u_base = obj.base_behavior.compute_control(swarm, env);
                u(:, obj.leader_idx) = u_base(:, obj.leader_idx);
            end

            follower_idx = 1;
            for i = 1:swarm.N
                if i == obj.leader_idx, continue; end

                agent = swarm.agents{i};
                pos   = agent.state(1:2);
                vel   = agent.state(3:4);

                if follower_idx <= size(obj.formation_offsets, 2)
                    desired_offset = obj.formation_offsets(:, follower_idx);
                else
                    angle          = 2*pi*follower_idx / swarm.N;
                    desired_offset = 5 * [cos(angle); sin(angle)];
                end

                desired_pos = leader_pos + desired_offset;
                pos_error   = desired_pos - pos;
                vel_error   = leader_vel  - vel;

                u(:,i) = obj.k_formation * pos_error + obj.k_velocity * vel_error;
                if norm(u(:,i)) > obj.max_accel
                    u(:,i) = u(:,i) / norm(u(:,i)) * obj.max_accel;
                end

                follower_idx = follower_idx + 1;
            end
        end
    end
end
