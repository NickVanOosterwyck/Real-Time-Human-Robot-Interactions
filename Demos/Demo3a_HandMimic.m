%% Add path
addpath(genpath(pwd)); % make sure current directory is the top map!

%% Clear
clear; close all; clc

%% Create & Connect
CameraType = 'real';    % vrep or real
RobotType = 'real';     % vrep or real

ctrl = controller(CameraType,RobotType);
ctrl.connect();

%% Hand Mimicking
a=0.5; v=0.2; t=0.16; r=0; la_t=0.2; gain=100;
%a=0.5; v=0.2; t=0; r=0;
LimitsX=[200 800];
LimitsZ=[200 800];

%% Go to start
ctrl.rob.movej([mean(LimitsX),-500,mean(LimitsZ),90,0,0],0.5,0.1,0,0,'World');
rob2.movej([mean(LimitsX),-500,mean(LimitsZ),90,0,0],0.5,0.1,0,0,'World');

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
    tic
    % get position
    HandPos = ctrl.cam.getHandPosition('Right','Robot');
    x=mean(LimitsX)+HandPos(1)-StartX;
    z=mean(LimitsZ)+HandPos(3)-StartZ;
    
    % move robot
%     time=toc(timer);
%     while time<0.2
%         time=toc(timer);
%     end
    if x>LimitsX(1) && x<LimitsX(2) && z>LimitsZ(1) && z<LimitsZ(2)
        ctrl.rob.servoj([x,-500,z,90,0,0],a,v,t,la_t,gain,'World');
        rob2.movej([x,-500,z,90,0,0],a,v,t,r,'World');
    else
        pos=ctrl.rob.getJointPositions();
        ctrl.rob.servoj(pos,a,v,t,la_t,gain);
    end
%     timer=tic;
    
    % check for exit
    h_left=ctrl.cam.getHandHeight('Skeleton','Left','Max');
    if h_left > 1.8
        flag=false;
    end
    toc
end

% go to start
ctrl.rob.movej([mean(LimitsX),-500,mean(LimitsZ),90,0,0],0.5,0.1,0,0,'World');
rob2.movej([mean(LimitsX),-500,mean(LimitsZ),90,0,0],0.5,0.1,0,0,'World');