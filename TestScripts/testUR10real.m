%% Add path
addpath(genpath(pwd)); % make sure current directory is the top map!

%% Clear
clear; close all; clc

%% Connect
%-- choose UR10
%rob=ur10core('vrep');
rob=ur10core('real');
rob.connect();

