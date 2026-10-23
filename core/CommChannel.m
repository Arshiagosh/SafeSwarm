classdef CommChannel < handle
    %% CommChannel — communication channel with configurable impairments
    %
    % Models realistic radio communication by adding range limits and
    % probabilistic packet loss. Use get_adjacency() as a drop-in
    % replacement for swarm.get_adjacency() when running noisy experiments.
    %
    % Example:
    %   ch  = CommChannel(15.0, 0.1);   % 15 m range, 10% packet loss
    %   adj = ch.get_adjacency(swarm);  % impaired adjacency matrix

    properties
        range       = Inf;  % maximum communication range (metres)
        packet_loss = 0.0;  % packet drop probability in [0, 1]
        delay_steps = 0;    % integer steps of communication delay (reserved)
        delay_buffer        % cell buffer for delayed states (reserved)
    end

    methods
        function obj = CommChannel(range, packet_loss, delay_steps)
            % CommChannel(range, packet_loss, delay_steps)
            %   range       : max comm range in metres (default Inf)
            %   packet_loss : drop probability in [0,1] (default 0)
            %   delay_steps : integer delay in steps (default 0, reserved)
            if nargin > 0, obj.range       = range;       end
            if nargin > 1, obj.packet_loss = packet_loss; end
            if nargin > 2
                obj.delay_steps  = delay_steps;
                obj.delay_buffer = {};
            end
        end

        function adj = get_adjacency(obj, swarm)
            % get_adjacency(swarm) — returns impaired N×N adjacency matrix
            %   Each link is sampled independently: in-range AND not dropped.
            N   = swarm.N;
            adj = zeros(N);
            for i = 1:N
                for j = 1:N
                    if i == j, continue; end
                    pi = swarm.agents{i}.state(1:2);
                    pj = swarm.agents{j}.state(1:2);
                    in_range = norm(pi - pj) <= obj.range;
                    not_lost = rand() > obj.packet_loss;
                    if in_range && not_lost
                        adj(i,j) = 1;
                    end
                end
            end
        end
    end
end
