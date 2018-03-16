classdef ur10real < handle
    %ur10real Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected)
        jpub5   % ???
        armur   % ???
        msgur   % ???
    end
    
    methods
        function obj = ur10real() % constructor   
        end
        
        function connectDif(obj)
            rosshutdown
            %rosinit('http://192.168.1.110:11311', 'NodeHost', '192.168.1.100') %see what is the port/IP number
            rosinit('http://192.168.1.100:11311', 'NodeHost', '192.168.1.16')
                        
            %[armur, msgur]=rosactionclient('/vel_based_pos_traj_controller/follow_joint_trajectory');
            %[armur, msgur]=rosactionclient('/pos_based_pos_traj_controller/follow_joint_trajectory');
            [obj.armur, obj.msgur]=rosactionclient('/follow_joint_trajectory');
            
            obj.jpub5=rossubscriber('/joint_states');
            pause(1)
            obj.msgur.Trajectory.JointNames={'shoulder_pan_joint', 'shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'}';
        end
        function [JointPositions] = getJointPositionsDif(obj)
            JointPositions_rad = obj.jpub5.LatestMessage.Position.';
            JointPositions=JointPositions_rad/pi*180;
            %JointPositions_rad(1) = obj.jpub5.LatestMessage.Position(3);
            %JointPositions_rad(3) = obj.jpub5.LatestMessage.Position(1);
        end
        function moveToJointTargetPositionsDif(obj,JointTargetPositions,MaxJointSpeedFactor)
            tjPoint2=rosmessage('trajectory_msgs/JointTrajectoryPoint');
            tjPoint2.Positions = JointTargetPositions.*pi./180;
            tjPoint2.Velocities = zeros(1,6);
                        
            tjPoint1=rosmessage('trajectory_msgs/JointTrajectoryPoint');
            tjPoint1.Velocities = zeros(1,6);
            tjPoint1.TimeFromStart = rosduration(0);
            tjPoint1.Positions = obj.jpub5.LatestMessage.Position;
            %tjPoint1.Positions(1) = obj.jpub5.LatestMessage.Position(3);
            %tjPoint1.Positions(3) = obj.jpub5.LatestMessage.Position(1);
            
            [IntTime] = obj.Positions2IntTime(tjPoint1.Positions./pi.*180,JointTargetPositions,MaxJointSpeedFactor);
            tjPoint2.TimeFromStart = rosduration(IntTime);
                        
            tjP=[tjPoint1, tjPoint2];
            obj.msgur.Trajectory.Points = tjP;
            sendGoal(obj.armur, obj.msgur); 
        end

    end
    methods (Static)
        function [IntTime] = Positions2IntTime(StartPositions,EndPositions,MaxJointSpeedFactor)
            deltaAng = abs(EndPositions-StartPositions);
            IntTime = max([deltaAng(1:2)./120 deltaAng(3:6)./180])/MaxJointSpeedFactor;
        end
    end
end

