%% Add path
addpath(genpath(pwd)); % make sure current directory is the top map!

%% Clear
clear; close all; clc

%% Create & Connect
CameraType = 'vrep';    % vrep or real
RobotType = 'vrep';     % vrep or real

ctrl = controller(CameraType,RobotType);
ctrl.connect();

%% Set up
%-- move camera
%ctrl.cam.moveToCameraLocation([2.03 2.03 1.08 90 -45 0]); % north-east

%-- set positions
Home = ctrl.rob.homeJointTargetPositions;
PickUp = [45 -110 -80 -170 -135 0];
PickUpApp = [45 -113.2953  -44.7716 -201.9331 -135 0];
Place = [-25 -110 -80 -170 -25 0];
PlaceApp = [-25 -113.2953  -44.7716 -201.9331 -25 0];

%-- create path
Path =[Home;PickUpApp;PickUp;PickUpApp;PlaceApp;Place;PlaceApp;Home];

%-- set safety distances
rStop = 1;

%% Go home
ctrl.rob.goHome(true);
disp('Robot is ready in home pose.')

%% Demo 1: Safety Rated Monitored Stop
MaxSpeedFactor = 1;
Range = 0.05;
treshold = 0.1;
iterations = 1;
Ref = 'TCP'; % choose TCP or Base
a=0.5; v=0.1; t=0; r=0;

ctrl.rob.setSpeedFactor(MaxSpeedFactor);
%rob2.setSpeedFactor(MaxSpeedFactor);
state = 0;
LastDist=Inf;
dis = GUI('ControlPanel',true,'LiveGraphDist',true);
dis.setValues('Reference',Ref,'SpeedFactor',MaxSpeedFactor);
tic
for it = 1:iterations
    i = 1;
    for i = 1:length(Path)
        state = 1;
        while ~ctrl.rob.checkPoseReached(Path(i,:),Range)
            [Dist,~,~] = ctrl.getClosestPoint(Ref);
            if Dist < rStop
                if state ~=0 && abs(LastDist-Dist)>treshold
                    LastDist = Dist;
                    ctrl.rob.stopj(a);
                    %rob2.stopj(a);
                    state = 0; disp('Robot is stopped')
                end
            else
                if  state ~=2 && (abs(LastDist-Dist)>treshold || state==1)
                    LastDist = Dist;
                    ctrl.rob.movel(Path(i,:),a,v,t,r);
                    %rob2.movej(Path(i,:),a,v,t,r);
                    state = 2; disp(['Target' num2str(i)])
                end
            end
            t=toc;
            dis.setValues('Dist',Dist,'LastDist',LastDist,'TargetPose',i,...
                'State',state,'Time',t);
        end
    end
end
close all
disp('End of loop reached')

