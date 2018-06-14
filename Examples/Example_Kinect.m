%% Clear
clear; close all; clc

%% Add path
addpath(genpath(pwd)); % make sure current directory is the top map!

%% Create & Connect
CameraType = 'vrep';    % vrep or real

cam=kinectcore(CameraType);
cam.connect();

%% Get Pointcloud
[ptCloud] = cam.getPointCloud('Filtered'); % Raw/Desampled/Filtered

%% Show Pointcloud
[ptCloud] = cam.getPointCloud('Filtered');
cam.createAxis();
pcshow(ptCloud);

%% Move camera
cam.moveToCameraLocation([-1.5 1.5 1 90 45 0]); % north-west


