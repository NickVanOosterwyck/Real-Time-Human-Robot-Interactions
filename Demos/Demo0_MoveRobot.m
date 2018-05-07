%% Add path
addpath(genpath(pwd)); % make sure current directory is the top map!

%% Clear
clear; close all; clc

%% Connect
CameraType = 'real';    % vrep or real
RobotType = 'real';     % vrep or real

ctrl = controller(CameraType,RobotType);
ctrl.connect();

%% 2nd robot
rob2=ur10core('vrep');
rob2.connect();

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
rob2.goHome();
ctrl.rob.goHome(true);
disp('Robot is ready in home pose.')

%% Demo 0: MoveRobot
MaxSpeedFactor = 1;
Range = 0.05;
iterations = 1;
a=0.5; v=0.2; t=0; r=0;

ctrl.rob.setSpeedFactor(MaxSpeedFactor);
rob2.setSpeedFactor(MaxSpeedFactor);
state = 0;
for it = 1:iterations
    i = 1;
    for i = 1:length(Path)
        state = 1;
        while ~ctrl.rob.checkPoseReached(Path(i,:),Range)
            if  state ==1
                ctrl.rob.movej(Path(i,:),a,v,t,r);
                rob2.movej(Path(i,:),a,v,t,r);
                state = 2;
            end
        end
    end
end
disp('End of loop reached')


