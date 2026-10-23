classdef DynamicObstacle < handle
    %% DynamicObstacle — a circular obstacle that moves and bounces off boundaries
    %
    % Call update(dt) each simulation step to advance the obstacle, then use
    % to_struct() to insert it into env.obstacles for collision checking.
    %
    % Example:
    %   obs = DynamicObstacle([0; 0], 3, [1; 0.5], [-50 50; -50 50]);
    %   for step = 1:n_steps
    %       obs.update(dt);
    %       env.obstacles{end} = obs.to_struct();
    %       sim.run_step();
    %   end

    properties
        center    % [x; y] current centre position
        radius    % scalar radius in metres
        velocity  % [vx; vy] current velocity (m/s)
        bounds    % 2×2 matrix: [xmin xmax; ymin ymax] bounce boundaries
    end

    methods
        function obj = DynamicObstacle(center, radius, velocity, bounds)
            % DynamicObstacle(center, radius, velocity, bounds)
            %   center   : [x; y] initial position
            %   radius   : scalar radius in metres
            %   velocity : [vx; vy] initial velocity (m/s)
            %   bounds   : 2×2 matrix [xmin xmax; ymin ymax]
            obj.center   = center;
            obj.radius   = radius;
            obj.velocity = velocity;
            obj.bounds   = bounds;
        end

        function update(obj, dt)
            % update(dt) — advance position and bounce off bounds
            obj.center = obj.center + obj.velocity * dt;
            for dim = 1:2
                if obj.center(dim) - obj.radius < obj.bounds(dim, 1)
                    obj.center(dim)   = obj.bounds(dim, 1) + obj.radius;
                    obj.velocity(dim) = -obj.velocity(dim);
                elseif obj.center(dim) + obj.radius > obj.bounds(dim, 2)
                    obj.center(dim)   = obj.bounds(dim, 2) - obj.radius;
                    obj.velocity(dim) = -obj.velocity(dim);
                end
            end
        end

        function obs_struct = to_struct(obj)
            % to_struct() — returns an obstacle struct compatible with Environment
            obs_struct.type   = 'circle';
            obs_struct.center = obj.center;
            obs_struct.radius = obj.radius;
        end
    end
end
