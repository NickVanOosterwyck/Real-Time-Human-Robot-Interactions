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
