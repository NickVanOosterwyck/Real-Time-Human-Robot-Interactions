classdef ur10vrep < handle
    %ur10vrep Summary of this class goes here
    %   Detailed explanation goes here
    
    properties %(SetAccess = protected)
        vrep            % object for communicating with VREP                [1x1 remApi]
        clientID;       % number of Matlab port (-1 if connection failed)   [1x1 double]
        JointHandles;   % handles of joints                                 [1x6 double]
    end
    
    methods
        function obj = ur10vrep() % constructor
            obj.vrep = remApi('remoteApi');
            obj.clientID=-1;
            obj.JointHandles = zeros(1,6);
        end
        
        function connectDif(obj)
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
        end
        function disconnectDif(obj)
            % Close connection
            obj.vrep.simxFinish(-1);
            
            % Call the destructor
            obj.vrep.delete(); clc;
        end
        function [JointPositions_rad] = getJointPositionsDif(obj)
            JointPositions_rad=zeros(1,6);
            [~]=obj.vrep.simxPauseCommunication(obj.clientID,1);
            for i=1:6
                [~,JointPositions_rad(i)]=obj.vrep.simxGetJointPosition(obj.clientID,obj.JointHandles(i),obj.vrep.simx_opmode_streaming);
            end
            [~]=obj.vrep.simxPauseCommunication(obj.clientID,0);
        end
        function moveToJointTargetPositionsDif(obj,JointTargetPositions)
            [~]=obj.vrep.simxPauseCommunication(obj.clientID,1);
            for i=1:6
                [~]=obj.vrep.simxSetJointTargetPosition(obj.clientID,obj.JointHandles(i),JointTargetPositions(i)*pi/180,obj.vrep.simx_opmode_streaming);
            end
            [~]=obj.vrep.simxPauseCommunication(obj.clientID,0);
        end
        function setMaxJointSpeedFactorDif(obj,MaxJointSpeedFactor)
            [~]=obj.vrep.simxPauseCommunication(obj.clientID,1);
            for i=1:2
                [~]=obj.vrep.simxSetJointTargetVelocity(obj.clientID,obj.JointHandles(i),120*MaxJointSpeedFactor*pi/180,obj.vrep.simx_opmode_streaming);
            end
            for i=3:4
                [~]=obj.vrep.simxSetJointTargetVelocity(obj.clientID,obj.JointHandles(i),180*MaxJointSpeedFactor*pi/180,obj.vrep.simx_opmode_streaming);
            end
            for i=5:6
                [~]=obj.vrep.simxSetJointTargetVelocity(obj.clientID,obj.JointHandles(i),min(180*MaxJointSpeedFactor*2,180)*pi/180,obj.vrep.simx_opmode_streaming);
            end
            [~]=obj.vrep.simxPauseCommunication(obj.clientID,0);

        end
        
        
    end
end

