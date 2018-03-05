classdef ur10vrep < ur10core
    %ur10vrep Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected)
        vrep            % object for communicating with VREP                [1x1 remApi]
        clientID;       % number of Matlab port (-1 if connection failed)   [1x1 double]
        JointHandles;   % handles of joints                                 [1x6 double]
    end
    
    methods
        function obj = ur10vrep() % constructor
            % initial values
            obj.clientID=-1;
            obj.JointHandles = zeros(1,6);
            
        end
        
        function connect(obj)
            % Using the prototype file (remoteApiProto.m)
            obj.vrep=remApi('remoteApi');clc;
            % Just in case, close all opened connections
            obj.vrep.simxFinish(-1);
            % Create a connection
            obj.clientID=obj.vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
            % Check connection
            if (obj.clientID>-1)
                disp('Connected to remote API server! (UR10)');
            else
                error('Problem with connection!!!\n%s','Make sure the simulation in VREP is running and try again.')
            end
            % get handles
            for i=1:6
                [~,obj.JointHandles(i)]=obj.vrep.simxGetObjectHandle(obj.clientID,['UR10_joint',num2str(i)],obj.vrep.simx_opmode_blocking);
            end
            % get positions
            [~] = obj.getJointPositions();
            pause(0.1)
            startPositions = obj.getJointPositions(); %2nd call because of streaming operation mode
            % set JointTargetPositions
            obj.moveToJointTargetPositions(startPositions);
        end
        function disconnect(obj)
            % Close connection
            obj.vrep.simxFinish(-1);
            
            % Call the destructor
            obj.vrep.delete(); clc;
        end
        function [JointPositions] = getJointPositions(obj)
            JointPositions_rad=zeros(1,6);
            [~]=obj.vrep.simxPauseCommunication(obj.clientID,1);
            for i=1:6
                [~,JointPositions_rad(i)]=obj.vrep.simxGetJointPosition(obj.clientID,obj.JointHandles(i),obj.vrep.simx_opmode_streaming);
            end
            [~]=obj.vrep.simxPauseCommunication(obj.clientID,0);
            JointPositions=round(JointPositions_rad/pi*180,2);
        end
        function moveToJointTargetPositions(obj,JointTargetPositions)
            obj.JointTargetPositions=JointTargetPositions;
            obj.TCPTargetPositions=obj.ForwKin(JointTargetPositions);
            
            [~]=obj.vrep.simxPauseCommunication(obj.clientID,1);
            for i=1:6
                [~]=obj.vrep.simxSetJointTargetPosition(obj.clientID,obj.JointHandles(i),JointTargetPositions(i)*pi/180,obj.vrep.simx_opmode_streaming);
            end
            [~]=obj.vrep.simxPauseCommunication(obj.clientID,0);
            
        end
        function goHome(obj)
            obj.moveToJointTargetPositions(obj.homeJointTargetPositions);
        end
        function stopRobot(obj)
            obj.moveToJointTargetPositions(obj.getJointPositions());
        end
        function setMaxJointSpeedFactor(obj,MaxJointSpeedFactor)
            obj.MaxJointSpeedFactor=MaxJointSpeedFactor;
            [~]=obj.vrep.simxPauseCommunication(obj.clientID,1);
            for i=1:2
                [~]=obj.vrep.simxSetJointTargetVelocity(obj.clientID,obj.JointHandles(i),120*obj.MaxJointSpeedFactor*pi/180,obj.vrep.simx_opmode_streaming);
            end
            for i=3:4
                [~]=obj.vrep.simxSetJointTargetVelocity(obj.clientID,obj.JointHandles(i),180*obj.MaxJointSpeedFactor*pi/180,obj.vrep.simx_opmode_streaming);
            end
            for i=5:6
                [~]=obj.vrep.simxSetJointTargetVelocity(obj.clientID,obj.JointHandles(i),min(180*obj.MaxJointSpeedFactor*2,180)*pi/180,obj.vrep.simx_opmode_streaming);
            end
            [~]=obj.vrep.simxPauseCommunication(obj.clientID,0);
            obj.moveToJointTargetPositions(obj.JointTargetPositions);
        end
        function [flag] = checkPoseReached(obj,JointTargetPositions)
            Positions = obj.getJointPositions();
            if max(abs(Positions-JointTargetPositions))< 0.5
                flag=1;
            else
                flag=0;
            end
        end
        
        
    end
end

