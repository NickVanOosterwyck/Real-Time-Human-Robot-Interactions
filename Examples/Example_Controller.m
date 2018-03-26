%% Clear
clear; close all; clc

%% Add path
addpath(genpath(pwd)); % make sure current directory is the top map!

%% Create & connect
CameraType = 'vrep';    % vrep or real
RobotType = 'vrep';     % vrep or real

ctrl = controller(CameraType,RobotType);
ctrl.connect();

%% Show Processed PointCloud
ctrl.showProcessedPointCloud('TCP'); % TCP or Base

%% Show Player
%ctrl.showPlayer(); %faster, but without tracking
ctrl.showTrackingPlayer('TCP'); % TCP or Base

%% Calculate closest distance
ptCloud = ctrl.cam.getPointCloud('Filtered');
[Dist,Start,End] = ctrl.getClosestPoint(ptCloud,'Base');
