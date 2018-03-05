%% Add path
addpath(genpath(pwd)); % make sure current directory is the top map!

%% Clear
clear; close all; clc

%% Connect
cam=kinectreal();
cam.connect();

%% Get filtered pointcloud
[ptCloudFiltered] = cam.getFilteredPointCloud();
cam.showPointCloud(ptCloudFiltered);

%% Get RGB images
