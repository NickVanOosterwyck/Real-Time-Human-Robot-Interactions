%% Clear
clear; close all; clc

%% Add path
addpath(genpath(pwd)); % make sure current directory is the top map!

%% Create & connect
cam=kinectvrep();   %cam=kinectreal();  % choose kinect
cam.connect();

%% Get raw pointcloud
[ptCloudRaw] = cam.getRawPointCloud();
cam.showPointCloud(ptCloudRaw);

%% Get desampled pointcloud
[ptCloudDesampled] = cam.getDesampledPointCloud();
cam.showPointCloud(ptCloudDesampled);

%% Get filtered pointcloud
[ptCloudFiltered] = cam.getFilteredPointCloud();
cam.showPointCloud(ptCloudFiltered);

%% Check pointclouds
% Show pointcloud calibration
[ptCloudRaw] = cam.getRawPointCloud();
cam.showPointCloudCalibration(ptCloudRaw);

% Show pointcloud comparison
[ptCloudDesampled] = cam.getDesampledPointCloud();
[ptCloudFiltered] = cam.getFilteredPointCloud();
cam.showPointCloudComparison(ptCloudDesampled,ptCloudFiltered);

%% Move camera
cam.moveToCameraLocation([0 2 1 90 0 0]);       % north
cam.moveToCameraLocation([3 0 1 0 -90 -90]);    % east
cam.moveToCameraLocation([0 -2 1 -90 0 -180]);  % south
cam.moveToCameraLocation([-2 0 1 90 0 0]);      % west

cam.moveToCameraLocation([-1.5 1.5 1 90 45 0]); % north-west

%% Disconnect
cam.disconnect();
