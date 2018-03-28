classdef ur10real < handle
    %ur10real Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected)
        jsub        % subscriber to joint states
        TCPsub      % subscriber to TCP velocity
        scriptpub   % publisher to URscript
        client      % rosactionclient to /follow_joint_trajectory 
        clientmsg   % message of rosactionclient to /follow_joint_trajectory 
    end
    
    methods
        function obj = ur10real() % constructor   
        end
        
        function connect(obj)
            rosshutdown
            rosinit('http://192.168.1.100:11311', 'NodeHost', '192.168.1.16')
            
            %[obj.client, obj.msgclient]=rosactionclient('/vel_based_pos_traj_controller/follow_joint_trajectory');
            %[obj.client, obj.msgclient=rosactionclient('/pos_based_pos_traj_controller/follow_joint_trajectory');
            [obj.client, obj.clientmsg]=rosactionclient('/follow_joint_trajectory');
            
            obj.jsub=rossubscriber('/joint_states');
            obj.TCPsub= rossubscriber('/tool_velocity');
            obj.scriptpub= rospublisher('/ur_driver/URScript','std_msgs/String');
            pause(1)
            obj.clientmsg.Trajectory.JointNames={'shoulder_pan_joint', 'shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'}';
        end
        function [JointPositions] = get_actual_joint_positions(obj)
            JointPositions = obj.jsub.LatestMessage.Position.';
        end
        function [Velocities] = get_actual_tcp_speed(obj)
            % only lineair speed
            vx = obj.TCPsub.LatestMessage.Twist.Linear.X;
            vy = obj.TCPsub.LatestMessage.Twist.Linear.Y;
            vz = obj.TCPsub.LatestMessage.Twist.Linear.Z;
            Velocities = [vx,vy,vz]; 
            
        end
        function movej(obj,q,a,v,t,r)
            msg=rosmessage('std_msgs/String');
            q_str = ['[' num2str(q(1)) ',' num2str(q(2)) ',' num2str(q(3)) ',' num2str(q(4)) ',' num2str(q(5)) ',' num2str(q(6)) ']'];
            msg.Data = ['movej(' q_str ',' num2str(a) ',' num2str(v) ',' num2str(t) ',' num2str(r) ')'];
            send(obj.scriptpub,msg); 
        end
        function movel(obj,q,a,v,t,r)
            msg=rosmessage('std_msgs/String');
            q_str = ['[' num2str(q(1)) ',' num2str(q(2)) ',' num2str(q(3)) ',' num2str(q(4)) ',' num2str(q(5)) ',' num2str(q(6)) ']'];
            msg.Data = ['movel(' q_str ',' num2str(a) ',' num2str(v) ',' num2str(t) ',' num2str(r) ')'];
            send(obj.scriptpub,msg);
        end
        function stopj(obj,a)
            msg=rosmessage('std_msgs/String');
            msg.Data = ['stopj(' num2str(a) ')'];
            send(obj.scriptpub,msg);
        end
        function setSpeedFactor(obj,SpeedFactor)
            msg=rosmessage('std_msgs/String');
            msg.Data = ['set speed' num2str(SpeedFactor)];
            send(obj.scriptpub,msg);
        end
        %{
        function moveToJointTargetPositions(obj,JointTargetPositions,MaxJointSpeedFactor)
            tjPoint2=rosmessage('trajectory_msgs/JointTrajectoryPoint');
            tjPoint2.Positions = JointTargetPositions.*pi./180;
            tjPoint2.Velocities = zeros(1,6);
                        
            tjPoint1=rosmessage('trajectory_msgs/JointTrajectoryPoint');
            tjPoint1.Velocities = zeros(1,6);
            tjPoint1.TimeFromStart = rosduration(0);
            tjPoint1.Positions = obj.jpub5.LatestMessage.Position;
            %tjPoint1.Positions(1) = obj.jsub.LatestMessage.Position(3);
            %tjPoint1.Positions(3) = obj.jsub.LatestMessage.Position(1);
            
            [IntTime] = obj.Positions2IntTime(tjPoint1.Positions./pi.*180,JointTargetPositions,MaxJointSpeedFactor);
            tjPoint2.TimeFromStart = rosduration(IntTime);
                        
            tjP=[tjPoint1, tjPoint2];
            obj.msgclient.Trajectory.Points = tjP;
            sendGoal(obj.client, obj.msgclient); 
        end
        %}
        
    end
    %{
    methods (Static)
        function [IntTime] = Positions2IntTime(StartPositions,EndPositions,MaxJointSpeedFactor)
            deltaAng = abs(EndPositions-StartPositions);
            IntTime = max([deltaAng(1:2)./120 deltaAng(3:6)./180])/MaxJointSpeedFactor;
        end

    end
    %}
end

