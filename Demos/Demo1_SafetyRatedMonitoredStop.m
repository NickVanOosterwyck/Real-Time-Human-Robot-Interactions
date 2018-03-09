%% Add path
addpath(genpath(pwd)); % make sure current directory is the top map!

%% Clear
clear; close all; clc

%% Connect
rob=ur10core('vrep');    %-- choose UR10
%rob=ur10core('real');
rob.connect();

cam=kinectcore('vrep');  %-- choose kinect
%cam=kinectcore('real');
cam.connect();

%% Set up
%-- move camera
cam.moveToCameraLocation([2.03 2.03 1.08 90 -45 0]); % north-east

%-- set positions
Home = rob.homeJointTargetPositions;
PickUp = [45 -125 -100 -135 -135 0];
PickUpApp = [45 -113.8520 -93.5075 -152.6405 -135 0];
Place = [-25 -125 -100 -135 -25 0];
PlaceApp = [-25 -113.8520 -93.5075 -152.6405 -25 0];

%-- create path
Path =[Home;PickUpApp;PickUp;PickUpApp;PlaceApp;Place;PlaceApp;Home];

%-- set safety distances
rStop = 2.65/2;

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
rob.goHome(0.1);
while ~rob.checkPoseReached(rob.homeJointTargetPositions)
end
disp('Robot is ready in home pose.')

%% Cycle
MaxSpeedFactor = 0.6;
iterations = 3;
flag = 0;

for it = 1:iterations
    i = 1;
    for i = 1:length(Path)
        flag = 1; disp('Next Target is set')
        while ~rob.checkPoseReached(Path(i,:))
            %tic
            [ptCloudFiltered] = cam.getFilteredPointCloud();
            [indices, dist] = findNeighborsInRadius(ptCloudFiltered,[0 0 1],5);
            %toc
            if ~isempty(indices) && min(dist)<rStop
                if flag ~=0 && (flag ==1 || flag ==2)
                rob.stopRobot();
                flag = 0; disp('Robot is stopped')
                end
            else
                if  flag ~=2 && (flag ==0 || flag ==1)
                rob.moveToJointTargetPositions(Path(i,:),MaxSpeedFactor);
                flag = 2; disp('Robot continues')
                end
            end
        end
    end
end

%% Flags
% 0	Stop
% 1	Next Target
% 2	Move normal
% 3	Move slow

