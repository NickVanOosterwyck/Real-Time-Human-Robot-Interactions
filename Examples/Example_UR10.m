%% Clear
clear; close all; clc

%% Add path
addpath(genpath(pwd)); % make sure current directory is the top map!

%% Create & Connect
RobotType = 'vrep';     % vrep or real

rob=ur10core(RobotType);
rob.connect();

%% Go home
rob.goHome();
rob.goHome(true); %return when robot is at pose

%% Movej
a=1.4; v=0.5; t=0; r=0;
rob.movej([45 -110 -80 -170 -135 0],a,v,t,r);       % with joint angles
rob.movej([500,-500,500,0,90,90],a,v,t,r,'World');  % in base coordinates
rob.movej([45 -110 -80 -170 -135 0],a,v,5,r);       % with specified time

%% Movel
a=1.4; v=0.5; t=0; r=0;
ctrl.rob.movel([45 -110 -80 -170 -135 0],a,v,t,r); % not supported in vrep

%% Show TCP coordinates
rob.TCPTargetPositions()

