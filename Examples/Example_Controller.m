%% Clear
clear; close all; clc

%% Add path
addpath(genpath(pwd)); % make sure current directory is the top map!

%% Create & connect
cam = 'vrep';
%cam = 'real';  % choose camera

rob = 'vrep';
%rob = 'real';  % choose ur10

ctrl = controller(cam,rob);
ctrl.connect();

%% Show Processed PointCloud
ctrl.showProcessedPointCloud();

%% Show Player
ctrl.showTrackingPlayerToTCP();

%% Calculate closest distance
ptCloud = ctrl.cam.getFilteredPointCloud();
[Dist1,Start1,End1] = ctrl.calculateClosestPoint(ptCloud,'Base');
[Dist2,Start2,End2] = ctrl.calculateClosestPoint(ptCloud,'TCP');
