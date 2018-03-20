%% Clear
clear; close all; clc

%% Add path
addpath(genpath(pwd)); % make sure current directory is the top map!

%% Create & Connect
CameraType = 'vrep';    % vrep or real

cam=kinectcore(CameraType);
cam.connect();

%% set detectionVol
cam.setdetecionVol([-2 1.5 -2 1.6 0 2.3]);

%% Get raw pointcloud
[ptCloudRaw] = cam.getRawPointCloud();
cam.plotPointCloud(ptCloudRaw);

%% Get filtered pointcloud
[ptCloudFiltered] = cam.getFilteredPointCloud();
cam.plotPointCloud(ptCloudFiltered);

%% Get desampled pointcloud
[ptCloudDesampled] = cam.getDesampledPointCloud();
cam.plotPointCloud(ptCloudDesampled);

%% Get comparison
cam.getPointCloudComparison();

%% Check calibration
cam.getPointCloudCalibration();

%% Show Player
cam.showPlayer();
cam.showTrackingPlayer(); %slower, but with live tracking

%% Move camera
cam.moveToCameraLocation([-1.5 1.5 1 90 45 0]); % north-west


