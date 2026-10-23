classdef Swarm < handle
    %% Swarm — collection of Agent objects with a communication topology
    %
    % Manages the adjacency graph used by behaviour algorithms to determine
    % which agents can communicate. Three topology modes are supported:
    %
    %   'metric' — agents within comm_radius are neighbours (default)
    %   'knn'    — each agent connects to its k_neighbours nearest agents
    %   'full'   — all-to-all connectivity (ignores comm_radius)
    %
    % The adjacency matrix is cached and invalidated automatically whenever
    % agent positions change (SimEngine calls invalidate_cache() each step).
    %
    % Example:
    %   agents = {Agent(1,[0;0;0;0], DoubleIntegrator()), ...};
    %   swarm  = Swarm(agents, 10.0, 'metric');
    %   A      = swarm.get_adjacency();

    properties
        agents       = {};      % cell array of Agent objects
        N            = 0;       % number of agents
        comm_radius  = 10.0;    % communication range (metres), used by 'metric' topology
        topology     = 'metric';% 'metric' | 'knn' | 'full'
        k_neighbours = 6;       % neighbours per agent, used by 'knn' topology
        adjacency_cache = [];   % cached N×N adjacency matrix
        cache_valid     = false;% flag: cache matches current positions
    end

    methods
        function obj = Swarm(agents, comm_radius, topology, k_neighbours)
            % Swarm(agents, comm_radius, topology, k_neighbours)
            %   All arguments are optional — construct empty and call add_agent().
            %   agents       : cell array of Agent objects
            %   comm_radius  : scalar (default 10.0)
            %   topology     : 'metric' | 'knn' | 'full' (default 'metric')
            %   k_neighbours : integer (default 6, only used for 'knn')
            if nargin >= 1 && ~isempty(agents)
                obj.agents = agents;
                obj.N      = length(agents);
            end
            if nargin >= 2 && ~isempty(comm_radius),  obj.comm_radius  = comm_radius;  end
            if nargin >= 3 && ~isempty(topology),     obj.topology     = topology;     end
            if nargin >= 4 && ~isempty(k_neighbours), obj.k_neighbours = k_neighbours; end
        end

        function add_agent(obj, agent)
            % add_agent(agent) — append an Agent to the swarm
            obj.N = obj.N + 1;
            obj.agents{obj.N} = agent;
            obj.invalidate_cache();
        end

        function remove_agent(obj, agent_id)
            % remove_agent(agent_id) — remove an agent by its numeric ID
            idx = [];
            for i = 1:obj.N
                if obj.agents{i}.id == agent_id
                    idx = i; break;
                end
            end
            if ~isempty(idx)
                obj.agents(idx) = [];
                obj.N = obj.N - 1;
                obj.invalidate_cache();
            end
        end

        function invalidate_cache(obj)
            % invalidate_cache() — force adjacency recompute on next get_adjacency()
            obj.cache_valid = false;
        end

        function positions = get_positions(obj)
            % get_positions() — returns 2×N matrix of agent [x; y] positions
            positions = zeros(2, obj.N);
            for i = 1:obj.N
                positions(:, i) = obj.agents{i}.position;
            end
        end

        function velocities = get_velocities(obj)
            % get_velocities() — returns 2×N matrix of agent [vx; vy] velocities
            velocities = zeros(2, obj.N);
            for i = 1:obj.N
                velocities(:, i) = obj.agents{i}.velocity;
            end
        end

        function A = get_adjacency(obj)
            % get_adjacency() — returns the N×N binary adjacency matrix
            %   Result is cached until invalidate_cache() is called.
            if obj.cache_valid && ~isempty(obj.adjacency_cache)
                A = obj.adjacency_cache;
                return;
            end

            n = obj.N;
            P = obj.get_positions();              % 2×n

            % Vectorized pairwise squared distances: D2(i,j) = ||p_i - p_j||^2
            sq = sum(P.^2, 1);                    % 1×n
            D2 = max(sq.' + sq - 2 * (P.' * P), 0);  % n×n, clamp tiny negatives

            switch obj.topology
                case 'metric'
                    A = double(D2 <= obj.comm_radius^2);
                    A(1:n+1:end) = 0;             % remove self-loops

                case 'knn'
                    D2(1:n+1:end) = inf;          % exclude self
                    [~, order] = sort(D2, 1);     % nearest agents per column
                    kk   = min(obj.k_neighbours, n - 1);
                    rows = order(1:kk, :);
                    cols = repmat(1:n, kk, 1);
                    A = zeros(n);
                    A(sub2ind([n n], rows(:), cols(:))) = 1;
                    A = max(A, A');               % symmetrize

                case 'full'
                    A = ones(n) - eye(n);
            end

            obj.adjacency_cache = A;
            obj.cache_valid     = true;
        end

        function neighbours = get_neighbours(obj, agent_idx)
            % get_neighbours(agent_idx) — indices of agents adjacent to agent_idx
            A          = obj.get_adjacency();
            neighbours = find(A(agent_idx, :));
        end
    end
end
