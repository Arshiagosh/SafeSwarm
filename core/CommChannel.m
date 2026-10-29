classdef CommChannel < handle
    %% CommChannel — what each robot knows about the others
    %
    % Every robot broadcasts its state once per control step. A message from
    % robot j to robot i arrives only if they are within range, it is not
    % dropped (probability packet_loss, independent per link and step) and j
    % is not silent. A message carries j's state from delay_steps steps ago.
    % Robot i keeps the last message it received from each j, so after drops
    % it acts on stale information.
    %
    % Call update(S) once per control step with the true states; view(i, S)
    % then returns the state matrix as robot i sees it (its own column is
    % exact). SafetyFilter in 'decentralised' mode uses this when its comm
    % property is set.
    %
    % SwarmSim's get_adjacency(swarm) is kept for the behaviours that use it.
    %
    % Example:
    %   ch = CommChannel(Inf, 0.3, 2);   % no range limit, 30 % loss, 2-step delay
    %   safe = SafetyFilter(nominal, 'Mode', 'decentralised');
    %   safe.comm = ch;

    properties
        range       = Inf;  % maximum communication range (metres)
        packet_loss = 0.0;  % packet drop probability in [0, 1]
        delay_steps = 0;    % age of every delivered state, in control steps
        silent      = [];   % 1×N logical, robots that send nothing (faults)
        history     = {};   % last delay_steps+1 true state matrices
        last_rx     = [];   % state_dim × N × N: last_rx(:, j, i) is what i holds about j
        age         = [];   % N × N, steps since i last heard from j
        n_sent      = 0;    % messages attempted
        n_lost      = 0;    % messages dropped (loss, range or silence)
    end

    methods
        function obj = CommChannel(range, packet_loss, delay_steps)
            % CommChannel(range, packet_loss, delay_steps)
            %   range       : max comm range in metres (default Inf)
            %   packet_loss : drop probability in [0,1] (default 0)
            %   delay_steps : integer delay in control steps (default 0)
            if nargin > 0, obj.range       = range;       end
            if nargin > 1, obj.packet_loss = packet_loss; end
            if nargin > 2, obj.delay_steps = delay_steps; end
        end

        function update(obj, S)
            % update(S) — one round of broadcasts; S is the true state_dim×N matrix
            n = size(S, 2);
            if isempty(obj.last_rx)
                % Start with exact knowledge, as if the robots had just synchronised.
                obj.last_rx = repmat(S, 1, 1, n);
                obj.age     = zeros(n);
            end
            if isempty(obj.silent), obj.silent = false(1, n); end
            obj.history{end+1} = S;
            if numel(obj.history) > obj.delay_steps + 1
                obj.history(1) = [];
            end
            sent = obj.history{1};                 % state from delay_steps steps ago

            P   = S(1:2, :);
            D   = sqrt((P(1,:)' - P(1,:)).^2 + (P(2,:)' - P(2,:)).^2);   % D(i, j)
            ok  = (D <= obj.range) & (rand(n) >= obj.packet_loss) & ~obj.silent;
            ok(1:n+1:end) = false;
            obj.age = obj.age + 1;
            for i = 1:n
                js = find(ok(i, :));
                obj.last_rx(:, js, i) = sent(:, js);
                obj.age(i, js) = obj.delay_steps;
            end
            obj.n_sent = obj.n_sent + n*(n-1);
            obj.n_lost = obj.n_lost + n*(n-1) - nnz(ok);
        end

        function V = view(obj, i, S)
            % view(i, S) — states as robot i knows them (own column from S)
            V = obj.last_rx(:, :, i);
            V(:, i) = S(:, i);
        end

        function reset(obj)
            % reset() — forget all messages and counters
            obj.history = {}; obj.last_rx = []; obj.age = [];
            obj.n_sent = 0; obj.n_lost = 0;
        end

        function adj = get_adjacency(obj, swarm)
            % get_adjacency(swarm) — impaired N×N adjacency matrix (SwarmSim API)
            %   Each link is sampled independently: in-range AND not dropped.
            P   = swarm.get_positions();
            n   = swarm.N;
            D   = sqrt((P(1,:)' - P(1,:)).^2 + (P(2,:)' - P(2,:)).^2);
            adj = double((D <= obj.range) & (rand(n) > obj.packet_loss));
            adj(1:n+1:end) = 0;
        end
    end
end
