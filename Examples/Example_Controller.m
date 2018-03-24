%% Clear
clear; close all; clc

%% Add path
addpath(genpath(pwd)); % make sure current directory is the top map!

%% Create & connect
% cam = 'vrep';
cam = 'real';  % choose camera

% rob = 'vrep';
rob = 'real';  % choose ur10

ctrl = controller(cam,rob);
ctrl.connect();

%% Show Processed PointCloud
pause(2)
ctrl.showProcessedPointCloud('TCP');
%ctrl.showProcessedPointCloud('Base');

%% Show Player
%ctrl.showPlayer(); %faster, but without tracking
ctrl.showTrackingPlayer('TCP');
%ctrl.showTrackingPlayer('Base');

%% Calculate closest distance
ptCloud = ctrl.cam.getPointCloud('Filtered');
[Dist,Start,End] = ctrl.getClosestPoint(ptCloud,'Base');
%[Dist,Start,End] = ctrl.calculateClosestPoint(ptCloud,'TCP');
