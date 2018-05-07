%% Add path
addpath(genpath(pwd)); % make sure current directory is the top map!

%% Clear
clear; close all; clc

%% Create & Connect
CameraType = 'real';    % vrep or real
RobotType = 'real';     % vrep or real

ctrl = controller(CameraType,RobotType);
ctrl.connect();

%% 2nd robot
rob2=ur10core('vrep');
rob2.connect();

%% Hand Mimicking
a=0.5; v=0.2; t=0; r=0;
LimitsX=[200 700];
LimitsZ=[300 700];

%% Go to start
ctrl.rob.movej([mean(LimitsX),-600,mean(LimitsZ),90,0,0],a,v,t,r,'World');
rob2.movej([mean(LimitsX),-600,mean(LimitsZ),90,0,0],a,v,t,r,'World');

%% Hand Mimicking
% get start position
flag = true;
while flag
    h_left=ctrl.cam.getHandHeight('Skeleton','Left','Max');
    if h_left > 1.8
        HandPos = ctrl.cam.getHandPosition('Right','Robot');
        StartX = HandPos(1);
        StartZ = HandPos(3);
        flag=false;
    end
end

% wait for lower hand
flag = true;
while flag
    h_left=ctrl.cam.getHandHeight('Skeleton','Left','Max');
    if h_left < 1.8
        flag=false;
    end
end

% mimick
flag = true;
while flag
    % get position
    HandPos = ctrl.cam.getHandPosition('Right','Robot');
    x=mean(LimitsX)+HandPos(1)-StartX;
    z=mean(LimitsZ)+HandPos(3)-StartZ;
    
    % move robot
    if x>LimitsX(1) && x<LimitsX(2) && z>LimitsZ(1) && z<LimitsZ(2)
        %ctrl.rob.movej([x,-600,z,90,0,0],a,v,t,r,'World');
        rob2.movej([x,-600,z,90,0,0],a,v,t,r,'World');
    end
    
    % check for stop
    h_left=ctrl.cam.getHandHeight('Skeleton','Left','Max');
    if h_left > 1.8
        flag=false;
    end
end