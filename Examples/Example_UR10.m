%% Clear
clear; close all; clc

%% Add path
addpath(genpath(pwd)); % make sure current directory is the top map!

%% Create & Connect
RobotType = 'vrep';     % vrep or real

rob=ur10core(RobotType);
rob.connect();

%% Go home
rob.goHome(0.5);

%% Go to joint position
rob.moveToJointTargetPositions([-25 -125 -100 -135 -25 0],0.5);

%% Go to TCP position
rob.moveToTCPTargetPositions([400 -400 400 90 0 0],0.5);

%% Show TCP coordinates
rob.TCPTargetPositions()

