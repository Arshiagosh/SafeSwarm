classdef PlannersTest < matlab.unittest.TestCase
    %% PlannersTest — planners find collision-free paths on a known map

    properties
        env
    end

    methods (TestMethodSetup)
        function make_map(tc)
            rng(3);
            tc.env = Environment([-20 20], [-20 20]);
            tc.env.add_rectangular_obstacle([-2 2], [-20 10]);   % wall with a gap at the top
            tc.env.add_circular_obstacle([10, -5], 3);
        end
    end

    methods (Test)
        function astarFindsPathAroundWall(tc)
            path = AStar(tc.env, 0.5).plan([-10; -10], [10; 5]);
            tc.verifyNotEmpty(path);
            tc.verifyLessThan(norm(path(:, end) - [10; 5]), 0.75);
            tc.verifyFalse(PlannersTest.hits(tc.env, path));
            tc.verifyGreaterThan(max(path(2, :)), 10);   % had to go over the wall
        end

        function rrtFindsPath(tc)
            path = RRT(tc.env, 3000, 1.0).plan([-10; -10], [10; 5]);
            tc.verifyNotEmpty(path);
            tc.verifyFalse(PlannersTest.hits(tc.env, path));
        end

        function potentialFieldOpenSpace(tc)
            env = Environment([-20 20], [-20 20]);
            env.add_circular_obstacle([0, 0.5], 2);
            path = PotentialField(env).plan([-10; 0], [10; 0], 3000, 0.05);
            tc.verifyLessThan(norm(path(:, end) - [10; 0]), 1.0);   % stops within 1 m by design
            tc.verifyFalse(PlannersTest.hits(env, path));
        end
    end

    methods (Static)
        function bad = hits(env, path)
            % True if any waypoint, or any point on the straight segments, is in collision.
            bad = false;
            for k = 1:size(path, 2) - 1
                for s = linspace(0, 1, 20)
                    if env.in_collision((1 - s) * path(:, k) + s * path(:, k + 1))
                        bad = true; return;
                    end
                end
            end
        end
    end
end
