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
a=1.4; v=0.2; t=0; r=0;
rob.movej([0 -90 -90 -180 -90 0],a,v,t,r);          % with joint angles
rob.movej([500,-400,800,0,90,90],a,v,t,r,'World');  % in base coordinates
rob.movej([0 -90 -90 -180 -90 0],a,v,5,r);          % with specified time

%% Movel
a=0.5; v=0.2; t=0; r=0;
rob.movel([500,-400,800,0,90,90],a,v,t,r,'World'); % not supported in vrep

%% Stopj
rob.stopj(10);

%% Set speed
rob.setSpeedFactor(1);

%% Show
Speed = rob.getTCPSpeed();
Positions = rob.getJointPositions();

