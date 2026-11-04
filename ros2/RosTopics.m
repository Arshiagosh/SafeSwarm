classdef RosTopics
    %% RosTopics — topic names, message types and conversions for the ROS 2 bridge
    %
    % Every robot lives in its own namespace (e.g. /robot_3) and uses:
    %   <ns>/odom         nav_msgs/Odometry           state (pose, and twist for velocity)
    %   <ns>/cmd_nominal  command from the robot's own controller
    %   <ns>/cmd_safe     command after the SafeSwarm filter
    % Commands are stamped so nodes can tell a fresh message from an old one:
    %   SingleIntegrator  geometry_msgs/TwistStamped  (linear.x, linear.y = velocity)
    %   Unicycle          geometry_msgs/TwistStamped  (linear.x = v, angular.z = omega)
    %   DoubleIntegrator  geometry_msgs/AccelStamped  (linear.x, linear.y = acceleration)
    % On a real robot <ns>/cmd_safe is what gets remapped to the driver's cmd_vel.

    methods (Static)
        function t = cmd_type(dyn)
            if isa(dyn, 'DoubleIntegrator')
                t = 'geometry_msgs/AccelStamped';
            else
                t = 'geometry_msgs/TwistStamped';
            end
        end

        function msg = fill_cmd(msg, dyn, u, t)
            % Write command u (2×1) and stamp t (s) into a command message.
            msg = RosTopics.set_stamp(msg, t);
            if isa(dyn, 'DoubleIntegrator')
                msg.accel.linear.x = u(1); msg.accel.linear.y = u(2);
            elseif isa(dyn, 'Unicycle')
                msg.twist.linear.x = u(1); msg.twist.angular.z = u(2);
            else
                msg.twist.linear.x = u(1); msg.twist.linear.y = u(2);
            end
        end

        function u = read_cmd(msg, dyn)
            % Command u (2×1) from a command message.
            if isa(dyn, 'DoubleIntegrator')
                u = [msg.accel.linear.x; msg.accel.linear.y];
            elseif isa(dyn, 'Unicycle')
                u = [msg.twist.linear.x; msg.twist.angular.z];
            else
                u = [msg.twist.linear.x; msg.twist.linear.y];
            end
        end

        function msg = fill_odom(msg, x, t)
            % Write a SafeSwarm state vector into nav_msgs/Odometry.
            msg = RosTopics.set_stamp(msg, t);
            msg.pose.pose.position.x = x(1);
            msg.pose.pose.position.y = x(2);
            if numel(x) == 3
                msg.pose.pose.orientation.z = sin(x(3) / 2);
                msg.pose.pose.orientation.w = cos(x(3) / 2);
            else
                msg.pose.pose.orientation.w = 1;
            end
            if numel(x) == 4
                msg.twist.twist.linear.x = x(3);
                msg.twist.twist.linear.y = x(4);
            end
        end

        function x = read_odom(msg, state_dim)
            % SafeSwarm state vector (state_dim×1) from nav_msgs/Odometry.
            p = msg.pose.pose;
            switch state_dim
                case 2, x = [p.position.x; p.position.y];
                case 3, x = [p.position.x; p.position.y; ...
                             2 * atan2(p.orientation.z, p.orientation.w)];
                case 4, x = [p.position.x; p.position.y; ...
                             msg.twist.twist.linear.x; msg.twist.twist.linear.y];
            end
        end

        function msg = set_stamp(msg, t)
            % Time t (s) into msg.header.stamp, field by field (keeps the message type).
            ns = round(t * 1e9);
            msg.header.stamp.sec     = int32(floor(ns / 1e9));
            msg.header.stamp.nanosec = uint32(ns - 1e9 * floor(ns / 1e9));
        end

        function t = seconds(stamp)
            t = double(stamp.sec) + double(stamp.nanosec) * 1e-9;
        end
    end
end
