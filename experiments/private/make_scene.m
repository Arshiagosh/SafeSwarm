function sc = make_scene(name, kind, seed)
    % make_scene(name, kind, seed) — benchmark scene with agents, map and goals
    %   name : 'swap' | 'crossing' | 'corridor'
    %   kind : 'si' (SingleIntegrator, 1 m/s) | 'di' (DoubleIntegrator, 1 m/s, 1 m/s²)
    %   seed : random seed for the small start offsets that break symmetry
    %
    %   swap     — 8 agents on a circle of radius 5 m swap to the opposite side.
    %   crossing — 4 agents go left to right while 4 go bottom to top.
    %   corridor — 3 agents each way pass through a 2 m gap between two walls;
    %              every agent is sent through the gap centre as a via point.
    % Every agent has radius 0.3 m.
    if nargin < 3, seed = 1; end
    rng(seed);
    switch name
        case 'swap'
            via = [];
            N   = 8;
            ang = 2*pi*(0:N-1)/N;
            P0  = 5 * [cos(ang); sin(ang)];
            G   = -P0;
            env = Environment([-7 7], [-7 7]);
        case 'crossing'
            via = [];
            y   = [-1.5 -0.5 0.5 1.5];
            P0  = [-6*ones(1,4), y; y, -6*ones(1,4)];
            G   = [ 6*ones(1,4), y; y,  6*ones(1,4)];
            env = Environment([-8 8], [-8 8]);
        case 'corridor'
            y   = [-2 0 2];
            P0  = [-6*ones(1,3), 6*ones(1,3); y, y];
            G   = [ 6*ones(1,3), -6*ones(1,3); y, y];
            env = Environment([-8 8], [-8 8]);
            env.add_rectangular_obstacle([-1 1], [1 8]);
            env.add_rectangular_obstacle([-1 1], [-8 -1]);
            via = zeros(2, 6);
        otherwise
            error('make_scene:Name', 'Unknown scene %s', name);
    end
    P0 = P0 + 0.05 * randn(size(P0));

    N = size(P0, 2);
    agents = cell(1, N);
    for i = 1:N
        if strcmp(kind, 'si')
            agents{i} = Agent(i, P0(:, i), SingleIntegrator(1.0));
        else
            agents{i} = Agent(i, [P0(:, i); 0; 0], DoubleIntegrator(1.0, 1.0));
        end
        agents{i}.color = hsv2rgb([(i-1)/N, 0.75, 0.9]);
    end
    swarm = Swarm(agents, 10, 'full');
    swarm.agent_radius = 0.3;

    sc = struct('name', name, 'kind', kind, 'swarm', swarm, 'env', env, 'goals', G, ...
                'via', via);
end
