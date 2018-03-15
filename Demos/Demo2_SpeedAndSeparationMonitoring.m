%% Add path
addpath(genpath(pwd)); % make sure current directory is the top map!

%% Clear
clear; close all; clc

%% Connect
%rob=ur10core('vrep');    %-- choose UR10
rob=ur10core('real');
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
rStop = 1;
rSlow = 1.7;

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
while ~rob.checkPoseReached(rob.homeJointTargetPositions,0.1)
end
disp('Robot is ready in home pose.')

%% Cycle
MaxSpeedFactor = 0.1;
Range = 0.1;
iterations = 1;

state = 0;
flag=0;
lastDist=Inf;
for it = 1:iterations
    i = 1;
    for i = 1:length(Path)
        state=1;
        while ~rob.checkPoseReached(Path(i,:),Range)
            %tic
            [dist,~] = cam.getClosestPoint()
            %toc
            if  dist<rStop
                if state ~=0 && (state ==1 || state ==2 || state ==3)
                rob.stopRobot(); disp('Robot is stopped')
                state=0;
                end
            elseif dist>rStop && dist<rSlow
                if  state ~=3 && (state ==0 || state ==1 || state ==2)
                    flag=0;
                end
                if abs(lastDist-dist)>0.4 || state==1
                    lastDist = dist;
                    Speedfactor = min(((dist-rStop)/(rSlow-rStop))*MaxSpeedFactor,MaxSpeedFactor);
                    if flag==1
                        Target = Path(i,:);
                        flag=0;
                    elseif flag==0
                        Target = Path(i,:)+0.1;
                        flag=1;
                    end
                    rob.moveToJointTargetPositions(Target,Speedfactor);
                end
                state=3;
            else
                if  state ~=2 && (state ==0 || state ==1 || state ==3)
                    rob.moveToJointTargetPositions(Path(i,:),MaxSpeedFactor);
                state=2;
                end
            end
        end
    end
end

%% States
% 0	Stop
% 1	Next Target
% 2	Move normal
% 3	Move slow

