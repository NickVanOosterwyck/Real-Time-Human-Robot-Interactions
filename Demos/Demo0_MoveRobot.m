%% Add path
addpath(genpath(pwd)); % make sure current directory is the top map!

%% Clear
clear; close all; clc

%% Connect
%rob=ur10core('vrep');    %-- choose UR10
rob=ur10core('real');
rob.connect();

%cam=kinectcore('vrep');  %-- choose kinect
cam=kinectcore('real');
cam.connect();

%% Set up
%-- set positions
Home = rob.homeJointTargetPositions;
PickUp = [20 -110 -80 -170 -135 0];
PickUpApp = [20 -113.2953  -44.7716 -201.9331 -135 0];
Place = [-20 -110 -80 -170 -25 0];
PlaceApp = [-20 -113.2953  -44.7716 -201.9331 -25 0];

%-- create path
Path =[Home;PickUpApp;PickUp;PickUpApp;PlaceApp;Place;PlaceApp;Home];

%% Go home
rob.goHome(0.05);
while ~rob.checkPoseReached(rob.homeJointTargetPositions,0.5)
end
disp('Robot is ready in home pose.')

%% Cycle
MaxSpeedFactor = 0.1;
Range = 0.1;
iterations = 1;

state = 0;
for it = 1:iterations
    i = 1;
    for i = 1:length(Path)
        state = 1;
        while ~rob.checkPoseReached(Path(i,:),Range)
            if  state ~=2 && state ==1
                rob.moveToJointTargetPositions(Path(i,:),MaxSpeedFactor);
                state = 2;
            end
            
        end
    end
end

