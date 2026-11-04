classdef RosBridgeTest < matlab.unittest.TestCase
    %% RosBridgeTest — message conversions of the ROS 2 bridge (no network needed)

    methods (TestClassSetup)
        function needsRos(tc)
            tc.assumeTrue(exist('ros2message', 'file') == 2, 'ROS Toolbox not installed');
        end
    end

    methods (Test)
        function odomRoundTrip(tc)
            m = ros2message('nav_msgs/Odometry');
            for x = {[1.5; -2], [1.5; -2; 2.5], [1.5; -2; 0.3; -0.4]}
                m2 = RosTopics.fill_odom(m, x{1}, 12.345);
                tc.verifyEqual(RosTopics.read_odom(m2, numel(x{1})), x{1}, 'AbsTol', 1e-12);
                tc.verifyEqual(RosTopics.seconds(m2.header.stamp), 12.345, 'AbsTol', 1e-9);
            end
        end

        function commandRoundTrip(tc)
            for dyn = {SingleIntegrator(1), Unicycle(1, 1), DoubleIntegrator(1, 1)}
                m = ros2message(RosTopics.cmd_type(dyn{1}));
                m = RosTopics.fill_cmd(m, dyn{1}, [0.3; -0.7], 4.2);
                tc.verifyEqual(RosTopics.read_cmd(m, dyn{1}), [0.3; -0.7], 'AbsTol', 1e-12);
            end
        end
    end
end
