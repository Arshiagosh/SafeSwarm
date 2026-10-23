classdef VirtualStructure < handle
    %% VirtualStructure — rigid virtual body formation control
    %
    % The swarm is treated as a single virtual rigid body. Each agent tracks
    % a point in the formation defined by formation_shape relative to a
    % virtual_center. The virtual center moves at virtual_velocity each step.
    %
    % NOTE: virtual_center is updated internally at a hardcoded 0.05 s step.
    % Set virtual_velocity accordingly or call set_virtual_velocity() at runtime.
    %
    % Works with: DoubleIntegrator
    % Use case:   coordinated rigid formations (V-shape, line, wedge)
    %
    % Example:
    %   shape = [-4 -2 0 2 4; 0 -2 0 -2 0];   % V-formation offsets
    %   vs    = VirtualStructure(shape, [0; 0]);
    %   vs.set_virtual_velocity([0.5; 0.5]);
    %   sim   = SimEngine(swarm, env, vs);

    properties
        formation_shape             % 2×N matrix of offsets from virtual_center
        k_formation  = 2.0;        % formation tracking gain
        k_consensus  = 1.0;        % consensus (centre alignment) gain
        virtual_center              % [x; y] current virtual body centre
        virtual_velocity = [0; 0]; % [vx; vy] virtual body velocity (m/s)
        max_accel    = 3.0;        % control saturation (m/s²)
    end

    methods
        function obj = VirtualStructure(shape, center)
            % VirtualStructure(shape, center)
            %   shape  : 2×N matrix of formation offsets
            %   center : [x; y] initial virtual centre
            obj.formation_shape = shape;
            obj.virtual_center  = center;
        end

        function set_virtual_velocity(obj, vel)
            % set_virtual_velocity(vel) — set [vx; vy] velocity of virtual body
            obj.virtual_velocity = vel;
        end

        function u = compute_control(obj, swarm, ~)
            % compute_control(swarm, env) — returns control_dim×N control matrix
            ctrl_dim = swarm.agents{1}.dynamics.control_dim;
            u        = zeros(ctrl_dim, swarm.N);

            current_center = mean(cell2mat(cellfun(@(a) a.state(1:2), ...
                                   swarm.agents, 'UniformOutput', false)), 2);
            center_error   = obj.virtual_center - current_center;

            for i = 1:swarm.N
                agent = swarm.agents{i};
                pos   = agent.state(1:2);

                if i <= size(obj.formation_shape, 2)
                    desired_offset = obj.formation_shape(:, i);
                else
                    angle          = 2*pi*i / swarm.N;
                    desired_offset = 5 * [cos(angle); sin(angle)];
                end

                desired_pos = obj.virtual_center + desired_offset;
                pos_error   = desired_pos - pos;

                u(:,i) = obj.k_formation * pos_error + ...
                         obj.k_consensus * center_error + ...
                         obj.virtual_velocity;

                if norm(u(:,i)) > obj.max_accel
                    u(:,i) = u(:,i) / norm(u(:,i)) * obj.max_accel;
                end
            end

            % Advance virtual centre (hardcoded at 0.05 s — matches default dt)
            obj.virtual_center = obj.virtual_center + obj.virtual_velocity * 0.05;
        end
    end
end
