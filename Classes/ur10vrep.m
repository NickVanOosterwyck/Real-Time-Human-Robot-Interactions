classdef ur10vrep < handle
    %ur10vrep Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected)
        vrep                    % object for communicating with VREP                [1x1 remApi]
        clientID;               % number of Matlab port (-1 if connection failed)   [1x1 double]
        JointHandles;           % handles of joints                                 [1x6 double]
        MaxSpeedFactor          % limiting joint speed factor
        JointVelocities         % velocity of joint in rad/s
        TCPHandle
    end
    
    methods
        function obj = ur10vrep() % constructor
            obj.vrep = remApi('remoteApi');
            obj.clientID=-1;
            obj.JointHandles = zeros(1,6);
            obj.MaxSpeedFactor = 1;
            obj.JointVelocities = zeros(1,6);
            obj.TCPHandle =0;
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
            [~,obj.TCPHandle] = obj.vrep.simxGetObjectHandle(obj.clientID,'TCP',obj.vrep.simx_opmode_blocking);
            % get jointpositions
            [~] = obj.get_actual_joint_positions();
            [~] = obj.get_actual_tcp_speed();
        end
        function [JointPositions] = get_actual_joint_positions(obj)
            JointPositions=zeros(1,6);
            [~]=obj.vrep.simxPauseCommunication(obj.clientID,1);
            for i=1:6
                [~,JointPositions(i)]=obj.vrep.simxGetJointPosition(obj.clientID,obj.JointHandles(i),obj.vrep.simx_opmode_streaming);
            end
            [~]=obj.vrep.simxPauseCommunication(obj.clientID,0);
        end
        function get_actual_joint_speeds(~)
            error('get_actual_joint_speeds is not supported in VREP')
        end
        function [Velocities] =get_actual_tcp_speed(obj)
            [~,Velocities,~]=obj.vrep.simxGetObjectVelocity(obj.clientID,obj.TCPHandle,obj.vrep.simx_opmode_streaming);
        end
        function movej(obj,q,~,v,t,~)
            % a and r are ignored in vrep
            deltaAng = abs(q-obj.get_actual_joint_positions());
            if t == 0
                t = max(deltaAng/v);
            end
            obj.JointVelocities = deltaAng/t;
            
            [~]=obj.vrep.simxPauseCommunication(obj.clientID,1);
            for i=1:6
                [~]=obj.vrep.simxSetJointTargetVelocity(obj.clientID,obj.JointHandles(i),obj.JointVelocities(i)*obj.MaxSpeedFactor,obj.vrep.simx_opmode_streaming);
            end
            for i=1:6
                [~]=obj.vrep.simxSetJointTargetPosition(obj.clientID,obj.JointHandles(i),q(i),obj.vrep.simx_opmode_streaming);
            end
            [~]=obj.vrep.simxPauseCommunication(obj.clientID,0);
        end
        function movel(~,varargin)
            warning('movel is not supported in VREP');
        end
        function servoj(~,varargin)
            warning('servoj is not supported in VREP');
        end
        function stopj(obj,~)
            obj.movej(obj.get_actual_joint_positions(),0,0,0.1,0);
        end
        function setSpeedFactor(obj,SpeedFactor)
            obj.MaxSpeedFactor = SpeedFactor;
            Velocities = max(obj.JointVelocities*obj.MaxSpeedFactor,0.01);
            [~]=obj.vrep.simxPauseCommunication(obj.clientID,1);
            for i=1:6
                [~]=obj.vrep.simxSetJointTargetVelocity(obj.clientID,obj.JointHandles(i),Velocities(i),obj.vrep.simx_opmode_streaming);
            end
            [~]=obj.vrep.simxPauseCommunication(obj.clientID,0);
 
        end

    end

end

