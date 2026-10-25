classdef EngineTest < matlab.unittest.TestCase
    %% EngineTest — engine, logger and environment bookkeeping

    methods (Test)
        function collisionsAreLoggedNotHidden(tc)
            % Two agents driven straight at each other must show a collision.
            swarm = Swarm({Agent(1, [-2; 0], SingleIntegrator(1)), ...
                           Agent(2, [ 2; 0], SingleIntegrator(1))}, 10, 'full');
            swarm.agent_radius = 0.25;
            env   = Environment([-5 5], [-5 5]);
            to_origin = FunctionBehaviour(@(swarm, ~) -swarm.get_positions());
            sim   = SimEngine(swarm, env, to_origin, 0.05, 3);
            sim.contact_mode = 'none';
            sim.logger = DataLogger();
            sim.run();
            s = sim.logger.summary();
            tc.verifyLessThan(s.min_distance, 0.5);
            tc.verifyGreaterThan(s.agent_collisions, 0);
        end

        function obstacleClearance(tc)
            env = Environment([-5 5], [-5 5]);
            env.add_circular_obstacle([0, 0], 1);
            env.add_rectangular_obstacle([2 3], [-1 1]);
            d = env.obstacle_distances([0 4 2.5; 3 0 3]);
            tc.verifyEqual(d(:, 1), [2; hypot(2, 2)], 'AbsTol', 1e-12);
            tc.verifyEqual(d(:, 2), [3; 1], 'AbsTol', 1e-12);
            tc.verifyEqual(d(:, 3), [hypot(2.5, 3) - 1; 2], 'AbsTol', 1e-12);
        end

        function dynamicObstacleMoves(tc)
            env = Environment([-10 10], [-10 10]);
            env.add_dynamic_obstacle(DynamicObstacle([0; 0], 1, [1; 0], [-10 10; -10 10]));
            swarm = Swarm({Agent(1, [5; 5], SingleIntegrator(1))});
            sim = SimEngine(swarm, env, Aggregation(0), 0.1, 2);
            sim.run();
            tc.verifyEqual(env.obstacles{1}.center(:), [2; 0], 'AbsTol', 1e-12);
        end

        function timeAndLogLength(tc)
            swarm = Swarm({Agent(1, [0; 0], SingleIntegrator(1))});
            sim = SimEngine(swarm, Environment([-1 1], [-1 1]), Aggregation(1), 0.1, 1);
            sim.logger = DataLogger();
            sim.run();
            tc.verifyEqual(sim.t, 1, 'AbsTol', 1e-12);
            tc.verifyEqual(swarm.time, 1, 'AbsTol', 1e-12);
            tc.verifyEqual(sim.logger.log_count, 11);   % initial condition + 10 steps
        end
    end
end
