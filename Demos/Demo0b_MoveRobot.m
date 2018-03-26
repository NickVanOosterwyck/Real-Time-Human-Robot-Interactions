%% Add path
addpath(genpath(pwd)); % make sure current directory is the top map!

%% Clear
clear; close all; clc

%% Connect
CameraType = 'vrep';    % vrep or real
RobotType = 'vrep';     % vrep or real

ctrl = controller(CameraType,RobotType);
ctrl.connect();

%% Set up
%-- move camera
%ctrl.cam.moveToCameraLocation([2.03 2.03 1.08 90 -45 0]); % north-east

%-- set positions
PickUp = [45 -110 -80 -170 -135 0];
PickUpApp = [45 -113.2953  -44.7716 -201.9331 -135 0];
Place = [-25 -110 -80 -170 -25 0];
PlaceApp = [-25 -113.2953  -44.7716 -201.9331 -25 0];

%-- create path
Path =[PickUpApp;PickUp;PickUpApp;PlaceApp;Place;PlaceApp];

%-- set safety distances
rStop = 1;

%% Go home
ctrl.rob.goHome(true);
disp('Robot is ready in home pose.')

%% Demo 0: MoveRobot
MaxSpeedFactor = 1;
Range = 0.25;
iterations = 1;
a=1.4; v=0.1; t=0; r=0;

ctrl.rob.setSpeedFactor(MaxSpeedFactor);
ctrl.rob.goHome(true);
state = 0;
for it = 1:iterations
    i = 1;
    for i = 1:length(Path)
        state = 1;
        while ~ctrl.rob.checkPoseReached(Path(i,:),Range)
            if  state ==1
                ctrl.rob.movej(Path(i,:),a,v,t,r);
                state = 2;
            end
        end
    end
end
ctrl.rob.goHome(true);
disp('End of loop reached')


