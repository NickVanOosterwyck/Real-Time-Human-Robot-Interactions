%% Clear
clear; close all; clc

%% Add path
addpath(genpath(pwd)); % make sure current directory is the top map!

%% Create & connect
rob=ur10core('vrep');   
%rob=ur10core('real');   % choose ur10
rob.connect();

%% Go home
rob.goHome(0.5);

%% Go to position
rob.moveToJointTargetPositions([-25 -125 -100 -135 -25 0],0.5);

%% Show TCP coordinates
rob.TCPTargetPositions()

