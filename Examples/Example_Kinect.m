%% Clear
clear; close all; clc

%% Add path
addpath(genpath(pwd)); % make sure current directory is the top map!

%% Create & connect
cam=kinectcore('vrep');
%cam=kinectcore('real');    % choose kinect
cam.connect();

%% Get raw pointcloud
[ptCloudRaw] = cam.getRawPointCloud();
cam.showPointCloud(ptCloudRaw);

%% Get filtered pointcloud
[ptCloudFiltered] = cam.getFilteredPointCloud();
cam.showPointCloud(ptCloudFiltered);

%% Get desampled pointcloud
[ptCloudDesampled] = cam.getDesampledPointCloud();
cam.showPointCloud(ptCloudDesampled);

%% Get comparison
cam.getPointCloudComparison();

%% Check calibration
cam.getPointCloudCalibration();

%% Show Player
cam.showPlayer();
cam.showTrackingPlayer(); %slower, but with live tracking

%% Move camera
cam.moveHome();
cam.moveToCameraLocation([-1.5 1.5 1 90 45 0]); % north-west


