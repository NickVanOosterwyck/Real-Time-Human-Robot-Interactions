%% Clear
clear; close all; clc

%% Add path
addpath(genpath(pwd)); % make sure current directory is the top map!

%% Create & connect
rob=ur10core('vrep');   
%rob=ur10core('real');   % choose ur10
rob.connect();

%% Go home
rob.goHome(0.05);

%% Go to position
rob.moveToJointTargetPositions([-25 -125 -100 -135 -25 0],0.01);

%% Show TCP coordinates
rob.TCPTargetPositions()

%% Follow path
MaxJointSpeedFactor = 0.05;

rob.goHome(MaxJointSpeedFactor);
while ~rob.checkPoseReached(rob.homeJointTargetPositions)
    disp('1')
end

rob.moveToJointTargetPositions([30 -90 -90 -180 -90 0],MaxJointSpeedFactor);
while ~rob.checkPoseReached([30 -90 -90 -180 -90 0])
    disp('2')
end

rob.moveToJointTargetPositions([50 -90 -90 -180 -90 0],MaxJointSpeedFactor);
while ~rob.checkPoseReached([50 -90 -90 -180 -90 0])
    disp('3')
end

rob.goHome(MaxJointSpeedFactor);
while ~rob.checkPoseReached(rob.homeJointTargetPositions)
    disp('4')
end
%%
rob.goHome(MaxJointSpeedFactor);
while ~rob.checkPoseReached(rob.homeJointTargetPositions)
    disp('1')
end
%% low
rob.moveToJointTargetPositions([45 -125 -100 -135 -135 0],MaxJointSpeedFactor);
while ~rob.checkPoseReached([45 -125 -100 -135 -135 0])
    disp('3')
end
%% 
pause(0.5)
rob.moveToJointTargetPositions([45 -113.8520 -93.5075 -152.6405 -135 0],MaxJointSpeedFactor);
while ~rob.checkPoseReached([45 -113.8520 -93.5075 -152.6405 -135 0])
    disp('4')
end
%%
rob.moveToJointTargetPositions([-25 -113.8520 -93.5075 -152.6405 -25 0],MaxJointSpeedFactor);
while ~rob.checkPoseReached([-25 -113.8520 -93.5075 -152.6405 -25 0])
    disp('5')
end
%% low
rob.moveToJointTargetPositions([-25 -125 -100 -135 -25 0],MaxJointSpeedFactor);
while ~rob.checkPoseReached([-25 -125 -100 -135 -25 0])
end
pause(0.5)
rob.moveToJointTargetPositions([-25 -113.8520 -93.5075 -152.6405 -25 0],MaxJointSpeedFactor);
while ~rob.checkPoseReached([-25 -113.8520 -93.5075 -152.6405 -25 0])
end
rob.goHome(MaxJointSpeedFactor);
while ~rob.checkPoseReached(rob.homeJointTargetPositions)
end

