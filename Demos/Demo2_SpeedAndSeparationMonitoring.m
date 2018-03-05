%% Add path
addpath(genpath(pwd)); % make sure current directory is the top map!

%% Clear
clear; close all; clc

%% Connect
%-- choose UR10
rob=ur10vrep();     %rob=ur10real();
rob.connect();

%-- choose kinect
cam=kinectvrep();   %cam=kinectreal();
cam.connect();

%% Set up
%-- move camera
cam.moveToCameraLocation([2.03 2.03 1.08 90 -45 0]); % north-west

%-- set positions
Home = rob.homeJointTargetPositions;
PickUp = [45 -125 -100 -135 -135 0];
PickUpApp = [45 -113.8520 -93.5075 -152.6405 -135 0];
Place = [-25 -125 -100 -135 -25 0];
PlaceApp = [-25 -113.8520 -93.5075 -152.6405 -25 0];

%-- create path
Path =[Home;PickUpApp;PickUp;PickUpApp;PlaceApp;Place;PlaceApp;Home];

%-- set safety distances
rStop = 1.35;
rSlow = 2.5;

%% Check pointclouds
% Show pointcloud calibration
[ptCloudRaw] = cam.getRawPointCloud();
cam.showPointCloudCalibration(ptCloudRaw);

% Show pointcloud comparison
[ptCloudDesampled] = cam.getDesampledPointCloud();
[ptCloudFiltered] = cam.getFilteredPointCloud();
cam.showPointCloudComparison(ptCloudDesampled,ptCloudFiltered);

%% Go home
% limit speed
rob.setMaxJointSpeedFactor(0.1);
rob.goHome();
while ~rob.checkPoseReached(rob.homeJointTargetPositions)
end
disp('Robot is ready in home pose.')

%% Cycle
iterations = 5;
speedlimit = 0.5;

for it = 1:iterations
    i = 1;
    for i = 1:length(Path)
        while ~rob.checkPoseReached(Path(i,:))
            [ptCloudFiltered] = cam.getFilteredPointCloud();
            [indices, dist] = findNeighborsInRadius(ptCloudFiltered,[0 0 1],5);
            if ~isempty(indices) && min(dist)<rStop
                [~] = rob.getJointPositions();
                rob.stopRobot();
            elseif ~isempty(indices) && min(dist)>rStop && min(dist)<rSlow
                Speedfactor = (min(dist)-rStop)/(rSlow-rStop)*speedlimit;
                rob.setMaxJointSpeedFactor(Speedfactor);
                rob.moveToJointTargetPositions(Path(i,:));
            else
                rob.setMaxJointSpeedFactor(1*speedlimit);
                rob.moveToJointTargetPositions(Path(i,:));
            end
        end
    end
end

%% Disconnect
rob.disconnect();
