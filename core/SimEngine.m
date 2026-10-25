classdef SimEngine < handle
    %% SimEngine — main simulation loop
    %
    % Ties together a Swarm, an Environment, and a Behaviour. Each step:
    %   1. Dynamic obstacles in the Environment move.
    %   2. Behaviour computes control inputs for all agents.
    %   3. Each agent advances its state via its dynamics model.
    %   4. Contact handling (see contact_mode).
    %   5. The adjacency cache is invalidated so topology updates next step.
    %   6. Metrics are logged and the visualizer is updated (if attached).
    %
    % contact_mode decides what happens when an agent reaches a wall or an
    % obstacle:
    %   'reflect' — SwarmSim behaviour: push the agent out and reflect its
    %               velocity with damping. Good for demos, but it hides
    %               collisions.
    %   'none'    — leave the state alone. Collisions stay visible in the
    %               logged metrics. Use this for every safety experiment.
    %
    % control_period sets how often the behaviour is evaluated. Empty (the
    % default) means every step. A larger value models a sampled-data
    % controller: the command is held constant (zero-order hold) for
    % control_period seconds while the dynamics keep integrating with dt.
    % control_period should be a multiple of dt.
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
        contact_mode = 'reflect';  % 'reflect' | 'none'
        control_period = [];       % zero-order-hold period (s); empty = every step
        u_last      % control applied in the last step (control_dim×N)
        t_control = -Inf;          % time the held command was computed
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
            if ~isempty(obj.logger) && obj.logger.log_count == 0
                obj.logger.log(obj.t, obj.swarm, obj.env);   % initial condition
            end
            for k = 1:n_steps
                obj.step();
            end
        end

        function step(obj)
            % step() — advance the whole simulation by one timestep dt
            obj.swarm.time = obj.t;
            obj.env.update_dynamic_obstacles(obj.dt);

            if isempty(obj.control_period) || isempty(obj.u_last) || ...
               obj.t - obj.t_control >= obj.control_period - 1e-9
                obj.u_last    = obj.behaviour.compute_control(obj.swarm, obj.env);
                obj.t_control = obj.t;
            end
            u_all = obj.u_last;

            for i = 1:obj.swarm.N
                obj.swarm.agents{i}.step(u_all(:,i), obj.dt);
                if strcmp(obj.contact_mode, 'reflect')
                    obj.enforce_bounds(obj.swarm.agents{i});
                    obj.enforce_obstacle_avoidance(obj.swarm.agents{i});
                end
            end

            obj.t = obj.t + obj.dt;
            obj.swarm.time = obj.t;
            obj.swarm.invalidate_cache();

            if ~isempty(obj.logger)
                obj.logger.log(obj.t, obj.swarm, obj.env, u_all);
            end

            if ~isempty(obj.visualizer)
                obj.visualizer.update(obj.swarm, obj.env, obj.t);
            end
        end

        function reset(obj)
            % reset() — set simulation time back to zero
            obj.t = 0;
            obj.swarm.time = 0;
            obj.u_last    = [];
            obj.t_control = -Inf;
        end
    end

    methods (Access = private)
        function enforce_bounds(obj, agent)
            % Reflect velocity at world boundaries with damping.
            pos       = agent.position;
            state_dim = length(agent.state);
            has_vel   = state_dim >= 4;

            if pos(1) < obj.env.x_lim(1)
                pos(1) = obj.env.x_lim(1);
                if has_vel, agent.state(3) = -obj.VELOCITY_DAMPING * agent.state(3); end
            elseif pos(1) > obj.env.x_lim(2)
                pos(1) = obj.env.x_lim(2);
                if has_vel, agent.state(3) = -obj.VELOCITY_DAMPING * agent.state(3); end
            end

            if pos(2) < obj.env.y_lim(1)
                pos(2) = obj.env.y_lim(1);
                if has_vel, agent.state(4) = -obj.VELOCITY_DAMPING * agent.state(4); end
            elseif pos(2) > obj.env.y_lim(2)
                pos(2) = obj.env.y_lim(2);
                if has_vel, agent.state(4) = -obj.VELOCITY_DAMPING * agent.state(4); end
            end

            agent.state(1:2) = pos;
        end

        function enforce_obstacle_avoidance(obj, agent)
            % Eject agent from obstacle interior; reflect velocity on contact.
            if isempty(obj.env.obstacles), return; end

            pos       = agent.position;
            has_vel   = length(agent.state) >= 4;
            margin    = 0.1;

            for k = 1:length(obj.env.obstacles)
                obs = obj.env.obstacles{k};

                if strcmp(obs.type, 'rect')
                    xr = obs.x_range;
                    yr = obs.y_range;
                    if pos(1) >= xr(1) && pos(1) <= xr(2) && ...
                       pos(2) >= yr(1) && pos(2) <= yr(2)

                        dists = [pos(1)-xr(1), xr(2)-pos(1), pos(2)-yr(1), yr(2)-pos(2)];
                        [~, edge] = min(dists);
                        switch edge
                            case 1, pos(1) = xr(1) - margin; c = 3; s = -1;
                            case 2, pos(1) = xr(2) + margin; c = 3; s =  1;
                            case 3, pos(2) = yr(1) - margin; c = 4; s = -1;
                            case 4, pos(2) = yr(2) + margin; c = 4; s =  1;
                        end
                        if has_vel
                            agent.state(c) = s * abs(agent.state(c)) * obj.VELOCITY_DAMPING;
                        end
                        agent.state(1:2) = pos;
                    end

                elseif strcmp(obs.type, 'circle')
                    d    = pos(:) - obs.center(:);
                    dist = norm(d);
                    if dist < obs.radius
                        if dist < 1e-6
                            n = [1; 0];
                        else
                            n = d / dist;
                        end
                        pos = obs.center(:) + (obs.radius + margin) * n;
                        agent.state(1:2) = pos;
                        if has_vel
                            vel = agent.state(3:4);
                            agent.state(3:4) = (vel - 2*dot(vel, n)*n) * obj.VELOCITY_DAMPING;
                        end
                    end
                end
            end
        end
    end
end
