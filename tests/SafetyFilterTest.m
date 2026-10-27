classdef SafetyFilterTest < matlab.unittest.TestCase
    %% SafetyFilterTest — the CBF-QP filter keeps its barriers non-negative

    methods (Test)
        function headOnPairStaysApart(tc)
            % Two agents driven at each other: the filter must keep D between them.
            for mode = ["centralised", "decentralised"]
                [swarm, env, nom] = SafetyFilterTest.head_on('si');
                f   = SafetyFilter(nom, 'Mode', char(mode));
                sim = SafetyFilterTest.simulate(swarm, env, f, 6);
                D   = f.safe_distance(swarm);
                tc.verifyGreaterThanOrEqual(sim.logger.summary().min_distance, D - 1e-3, char(mode));
                tc.verifyEqual(f.n_infeasible, 0);
            end
        end

        function centralisedMatchesProjection(tc)
            % With a single active pair constraint the QP has a closed form:
            % u = u_hat - a (a'u_hat - b)/||a||^2 when a'u_hat > b.
            [swarm, env, nom] = SafetyFilterTest.head_on('si');
            swarm.agents{1}.state = [-0.4; 0];
            swarm.agents{2}.state = [ 0.4; 0];
            f = SafetyFilter(nom, 'Mode', 'centralised', 'DSafe', 0.7, 'Alpha', 1);
            u = f.compute_control(swarm, env);
            uh = nom.compute_control(swarm, env);
            dp = [-0.8; 0];
            a  = [-2*dp; 2*dp];                  % row over [u_1; u_2]
            b  = 1 * (dp'*dp - 0.7^2);
            x  = uh(:) - a * (a'*uh(:) - b) / (a'*a);
            % The nominal command may also hit the input polygon; keep it inside here.
            tc.assumeLessThan(max(vecnorm(reshape(x, 2, 2))), 1.0 * cos(pi/16));
            tc.verifyEqual(u(:), x, 'AbsTol', 1e-6);
        end

        function decentralisedSplitsTheConstraint(tc)
            % Each agent's own half-row must hold, so the full row holds.
            [swarm, env, nom] = SafetyFilterTest.head_on('si');
            swarm.agents{1}.state = [-0.5; 0.05];
            swarm.agents{2}.state = [ 0.5; 0];
            f  = SafetyFilter(nom, 'Mode', 'decentralised', 'Alpha', 1);
            u  = f.compute_control(swarm, env);
            P  = swarm.get_positions();
            dp = P(:,1) - P(:,2);
            D  = f.safe_distance(swarm);
            h  = dp'*dp - D^2;
            tc.verifyLessThanOrEqual(-2*dp'*u(:,1), h/2 + 1e-7);
            tc.verifyLessThanOrEqual( 2*dp'*u(:,2), h/2 + 1e-7);
        end

        function doubleIntegratorStaysApart(tc)
            [swarm, env, nom] = SafetyFilterTest.head_on('di');
            f   = SafetyFilter(nom, 'Mode', 'centralised');
            sim = SafetyFilterTest.simulate(swarm, env, f, 10);
            tc.verifyGreaterThanOrEqual(sim.logger.summary().min_distance, ...
                                        f.safe_distance(swarm) - 1e-2);
        end

        function speedBarrierHolds(tc)
            [swarm, env, nom] = SafetyFilterTest.head_on('di');
            for i = 1:2, swarm.agents{i}.dynamics.max_speed = 0.5; end
            f   = SafetyFilter(nom, 'Mode', 'decentralised');
            SafetyFilterTest.simulate(swarm, env, f, 10);
            tc.verifyEqual(swarm.agents{1}.dynamics.speed_clips, 0);
            tc.verifyEqual(swarm.agents{2}.dynamics.speed_clips, 0);
        end

        function obstacleAvoided(tc)
            swarm = Swarm({Agent(1, [-4; 0.05], SingleIntegrator(1))}, 10, 'full');
            swarm.agent_radius = 0.3;
            env = Environment([-6 6], [-6 6]);
            env.add_circular_obstacle([0, 0], 1);
            f   = SafetyFilter(GoToGoal([4; 0]), 'Mode', 'decentralised');
            sim = SafetyFilterTest.simulate(swarm, env, f, 15);
            s   = sim.logger.summary();
            tc.verifyGreaterThanOrEqual(s.min_clearance, -1e-3);
            tc.verifyEqual(s.obstacle_collisions, 0);
        end

        function unicycleLookaheadStaysApart(tc)
            a1 = Agent(1, [-3; 0.05; 0],  Unicycle(1, 2));
            a2 = Agent(2, [ 3; 0;    pi], Unicycle(1, 2));
            swarm = Swarm({a1, a2}, 10, 'full');
            swarm.agent_radius = 0.3;
            f   = SafetyFilter(GoToGoal([3 -3; 0 0]), 'Mode', 'centralised');
            sim = SafetyFilterTest.simulate(swarm, Environment([-6 6], [-6 6]), f, 10);
            tc.verifyGreaterThanOrEqual(sim.logger.summary().min_distance, 0.6);
        end

        function infeasibilityIsReported(tc)
            % Start closer than D with a tiny input bound: the QP must report slack.
            a1 = Agent(1, [-0.2; 0], SingleIntegrator(0.01));
            a2 = Agent(2, [ 0.2; 0], SingleIntegrator(0.01));
            swarm = Swarm({a1, a2}, 10, 'full');
            swarm.agent_radius = 0.3;
            f = SafetyFilter(GoToGoal([1 -1; 0 0]), 'Mode', 'centralised');
            f.compute_control(swarm, []);
            tc.verifyEqual(f.n_infeasible, 1);
            tc.verifyGreaterThan(f.log_slack(end), 0);
        end
    end

    methods (Static)
        function [swarm, env, nom] = head_on(kind)
            if strcmp(kind, 'si')
                a1 = Agent(1, [-3; 0.05], SingleIntegrator(1));
                a2 = Agent(2, [ 3; 0],    SingleIntegrator(1));
            else
                a1 = Agent(1, [-3; 0.05; 0; 0], DoubleIntegrator(1, 1));
                a2 = Agent(2, [ 3; 0;    0; 0], DoubleIntegrator(1, 1));
            end
            swarm = Swarm({a1, a2}, 10, 'full');
            swarm.agent_radius = 0.3;
            env = Environment([-6 6], [-6 6]);
            nom = GoToGoal([3 -3; 0 0], 1.0);
        end

        function sim = simulate(swarm, env, behav, t_max)
            sim = SimEngine(swarm, env, behav, 0.01, t_max);
            sim.contact_mode = 'none';
            sim.logger = DataLogger();
            sim.run();
        end
    end
end
