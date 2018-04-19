%% Clear
clear; close all; clc

%% Add path
addpath(genpath(pwd)); % make sure current directory is the top map!

%% Create & connect
CameraType = 'real';    % vrep or real
RobotType = 'real';     % vrep or real

ctrl = controller(CameraType,RobotType);
ctrl.connect();

%% Show Distance Calculation
ctrl.showDistanceCalculation('Skeleton','TCP'); % ptCloud/Skeleton and TCP/Base

%% Show Player
ctrl.showTrackingPlayer('ptCloud','TCP'); % ptCloud/Skeleton and TCP/Base

%% Calculate closest distance
ptCloud = ctrl.cam.getPointCloud('Filtered');
[Dist,Start,End] = ctrl.getClosestPoint('Skeleton','Base');
