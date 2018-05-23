%% Clear
clear; close all; clc

%% Add path
addpath(genpath(pwd)); % make sure current directory is the top map!

%% Create & Connect
CameraType = 'real';    % vrep or real

cam=kinectcore(CameraType);
cam.connect();

%% set detectionVol
cam.setdetecionVol([-2 1.5 -2 1.6 0 2.3]);

%% Get Pointcloud
[ptCloud] = cam.getPointCloud('Filtered');
cam.plotPointCloud(ptCloud);

%% Get comparison
cam.getPointCloudComparison();

%% Show Player
cam.showTrackingPlayer();

%% Move camera
cam.moveToCameraLocation([-1.5 1.5 1 90 45 0]); % north-west


