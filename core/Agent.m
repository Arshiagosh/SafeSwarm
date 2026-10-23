classdef Agent < handle
    %% Agent — a single robot in the swarm
    %
    % Wraps a state vector and a dynamics model. State format depends on the
    % dynamics type assigned at construction:
    %
    %   SingleIntegrator : [x; y]              (2D)
    %   DoubleIntegrator : [x; y; vx; vy]      (4D)
    %   Unicycle         : [x; y; theta]        (3D)
    %
    % Example:
    %   a = Agent(1, [0; 0; 0; 0], DoubleIntegrator());
    %   a.step([1; 0], 0.1);   % apply control u=[1;0] for dt=0.1 s

    properties
        id          % unique agent identifier (integer)
        state       % state column vector (format depends on dynamics)
        dynamics    % dynamics model object (SingleIntegrator | DoubleIntegrator | Unicycle)
        color       % RGB triplet used by SwarmVisualizer
    end

    properties (Dependent)
        position    % [x; y] — first two elements of state
        velocity    % [vx; vy] — elements 3:4 of state, or [0;0] if unavailable
    end

    methods
        function obj = Agent(id, initial_state, dynamics)
            % Agent(id, initial_state, dynamics)
            %   id            : positive integer, unique within the swarm
            %   initial_state : column vector matching dynamics state format
            %   dynamics      : dynamics model object
            obj.id       = id;
            obj.state    = initial_state(:);
            obj.dynamics = dynamics;
            obj.color    = rand(1, 3);
        end

        function step(obj, u, dt)
            % step(u, dt) — advance agent state by one timestep
            %   u  : control input column vector (format matches dynamics)
            %   dt : timestep in seconds
            obj.state = obj.dynamics.step(obj.state, u, dt);
        end

        function pos = get.position(obj)
            pos = obj.state(1:2);
        end

        function vel = get.velocity(obj)
            if length(obj.state) >= 4
                vel = obj.state(3:4);
            else
                vel = [0; 0];
            end
        end

        function set.position(obj, pos)
            obj.state(1:2) = pos(:);
        end
    end
end
