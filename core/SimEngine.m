classdef SimEngine < handle
    %% SimEngine — main simulation loop
    %
    % Ties together a Swarm, an Environment, and a Behaviour. Each step:
    %   1. Behaviour computes control inputs for all agents.
    %   2. Each agent advances its state via its dynamics model.
    %   3. Boundary reflection and obstacle ejection are enforced.
    %   4. The adjacency cache is invalidated so topology updates next step.
    %   5. Metrics are logged and the visualizer is updated (if attached).
    %
    % Optional components are attached after construction:
    %   sim.logger     = DataLogger();
    %   sim.visualizer = SwarmVisualizer(swarm, env);
    %
    % Example:
    %   sim = SimEngine(swarm, env, behaviour, 0.1, 60);
    %   sim.logger = DataLogger();
    %   sim.run();

    properties
        swarm       % Swarm object
        env         % Environment object
        behaviour   % behaviour object (must implement compute_control)
        dt          % timestep in seconds (default 0.1)
        t_max       % maximum simulation time in seconds (default 60)
        t           % current simulation time
        logger      % DataLogger object (optional)
        visualizer  % SwarmVisualizer object (optional)
    end

    properties (Constant)
        VELOCITY_DAMPING = 0.8;  % energy loss coefficient on boundary/obstacle collision
    end

    methods
        function obj = SimEngine(swarm, env, behaviour, dt, t_max)
            % SimEngine(swarm, env, behaviour, dt, t_max)
            %   swarm     : Swarm object
            %   env       : Environment object
            %   behaviour : behaviour object
            %   dt        : timestep in seconds (default 0.1)
            %   t_max     : simulation duration in seconds (default 60)
            obj.swarm     = swarm;
            obj.env       = env;
            obj.behaviour = behaviour;
            obj.dt    = 0.1;
            obj.t_max = 60;
            if nargin >= 4 && ~isempty(dt),    obj.dt    = dt;    end
            if nargin >= 5 && ~isempty(t_max), obj.t_max = t_max; end
            obj.t          = 0;
            obj.logger     = [];
            obj.visualizer = [];
        end

        function run(obj, t_max_override, dt_override)
            % run() — execute the simulation loop
            %   run()             uses obj.dt and obj.t_max
            %   run(t_max)        overrides t_max
            %   run(t_max, dt)    overrides both (backward-compatible form)
            if nargin >= 2 && ~isempty(t_max_override), obj.t_max = t_max_override; end
            if nargin >= 3 && ~isempty(dt_override),    obj.dt    = dt_override;    end

            n_steps = round(obj.t_max / obj.dt);

            for step = 1:n_steps
                obj.t = obj.t + obj.dt;

                u_all = obj.behaviour.compute_control(obj.swarm, obj.env);

                for i = 1:obj.swarm.N
                    obj.swarm.agents{i}.step(u_all(:,i), obj.dt);
                    obj.enforce_bounds(obj.swarm.agents{i});
                    obj.enforce_obstacle_avoidance(obj.swarm.agents{i});
                end

                obj.swarm.invalidate_cache();

                if ~isempty(obj.logger)
                    obj.logger.log(obj.t, obj.swarm);
                end

                if ~isempty(obj.visualizer)
                    obj.visualizer.update(obj.swarm, obj.env, obj.t);
                    drawnow limitrate;
                end
            end
        end

        function reset(obj)
            % reset() — set simulation time back to zero
            obj.t = 0;
        end
    end

    methods (Access = private)
        function enforce_bounds(obj, agent)
            % Reflect velocity at world boundaries with damping.
            pos       = agent.position;
            state_dim = length(agent.state);

            if pos(1) < obj.env.x_lim(1)
                pos(1) = obj.env.x_lim(1);
                if state_dim >= 3, agent.state(3) = -obj.VELOCITY_DAMPING * agent.state(3); end
            elseif pos(1) > obj.env.x_lim(2)
                pos(1) = obj.env.x_lim(2);
                if state_dim >= 3, agent.state(3) = -obj.VELOCITY_DAMPING * agent.state(3); end
            end

            if pos(2) < obj.env.y_lim(1)
                pos(2) = obj.env.y_lim(1);
                if state_dim >= 4, agent.state(4) = -obj.VELOCITY_DAMPING * agent.state(4); end
            elseif pos(2) > obj.env.y_lim(2)
                pos(2) = obj.env.y_lim(2);
                if state_dim >= 4, agent.state(4) = -obj.VELOCITY_DAMPING * agent.state(4); end
            end

            agent.state(1:2) = pos;
        end

        function enforce_obstacle_avoidance(obj, agent)
            % Eject agent from obstacle interior; reflect velocity on contact.
            if isempty(obj.env.obstacles), return; end

            pos = agent.position;

            for k = 1:length(obj.env.obstacles)
                obs      = obj.env.obstacles{k};
                obs_type = lower(obs.type);
                margin   = 0.1;
                state_dim = length(agent.state);

                if strcmp(obs_type, 'rect')
                    if isfield(obs, 'x_range') && isfield(obs, 'y_range')
                        xr = obs.x_range;
                        yr = obs.y_range;
                    elseif isfield(obs, 'bounds')
                        xr = obs.bounds(1:2);
                        yr = obs.bounds(3:4);
                    else
                        continue;
                    end

                    if pos(1) >= xr(1) && pos(1) <= xr(2) && ...
                       pos(2) >= yr(1) && pos(2) <= yr(2)

                        dists = [pos(1)-xr(1), xr(2)-pos(1), pos(2)-yr(1), yr(2)-pos(2)];
                        [~, edge] = min(dists);

                        switch edge
                            case 1
                                pos(1) = xr(1) - margin;
                                if state_dim >= 3
                                    agent.state(3) = -abs(agent.state(3)) * obj.VELOCITY_DAMPING;
                                end
                            case 2
                                pos(1) = xr(2) + margin;
                                if state_dim >= 3
                                    agent.state(3) =  abs(agent.state(3)) * obj.VELOCITY_DAMPING;
                                end
                            case 3
                                pos(2) = yr(1) - margin;
                                if state_dim >= 4
                                    agent.state(4) = -abs(agent.state(4)) * obj.VELOCITY_DAMPING;
                                end
                            case 4
                                pos(2) = yr(2) + margin;
                                if state_dim >= 4
                                    agent.state(4) =  abs(agent.state(4)) * obj.VELOCITY_DAMPING;
                                end
                        end
                        agent.state(1:2) = pos;
                    end

                elseif strcmp(obs_type, 'circle')
                    diff = pos(:) - obs.center(:);
                    dist = norm(diff);

                    if dist < obs.radius
                        if dist < 1e-6
                            direction = [1; 0];
                        else
                            direction = diff / dist;
                        end
                        pos = obs.center(:) + (obs.radius + margin) * direction;
                        agent.state(1:2) = pos;

                        if state_dim >= 4
                            vel            = agent.state(3:4);
                            vel_normal     = dot(vel, direction) * direction;
                            agent.state(3:4) = (vel - 2*vel_normal) * obj.VELOCITY_DAMPING;
                        end
                    end
                end
            end
        end
    end
end
