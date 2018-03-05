%% Add path
addpath(genpath(pwd)); % make sure current directory is the top map!

%% Clear
clear; close all; clc

%% Connect
cam=kinectreal();
cam.connect();

%% Get ptCloud
%XYZ = cam.GetFrame(TofFrameType.XYZ_3_COLUMNS);
%ptCloud = pointCloud(XYZ);
ptCloud = cam.GetFrame(TofFrameType.POINTCLOUD_COLOR);
pcshow(ptCloud);