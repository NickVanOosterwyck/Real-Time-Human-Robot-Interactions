%% Clear
clear; close all; clc

%% Add path
addpath(genpath(pwd)); % make sure current directory is the top map!

%% Create & connect
CameraType = 'vrep';    % vrep or real
RobotType = 'vrep';     % vrep or real

ctrl = controller(CameraType,RobotType);
ctrl.connect();

%% set up
a=5; v=0.1; t=0; r=0;
rob.movej([45 -110 -80 -170 -135 0],a,v,t,r);

%% record
rob.movej([45 -110 -80 -170 -135 0],a,v,t,r);
ctrl.showTrackingPlayer('ptCloud','TCP'); % ptCloud/Skeleton and TCP/Base
