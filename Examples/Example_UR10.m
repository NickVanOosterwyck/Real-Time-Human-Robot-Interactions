%% Clear
clear; close all; clc

%% Add path
addpath(genpath(pwd)); % make sure current directory is the top map!

%% Create & connect
rob=ur10core('vrep');   %rob=ur10core('real');   % choose ur10
rob.connect();

%% Go home
rob.goHome();

%% Limit speed
rob.setMaxJointSpeedFactor(0.4);

%% Go to position
rob.moveToJointTargetPositions([-25 -125 -100 -135 -25 0]);

%% Show TCP coordinates
rob.TCPTargetPositions

%% Follow path
rob.setMaxJointSpeedFactor(0.6);
rob.goHome();
while ~rob.checkPoseReached(rob.homeJointTargetPositions)
end
rob.moveToJointTargetPositions([45 -113.8520 -93.5075 -152.6405 -135 0]);
while ~rob.checkPoseReached([45 -113.8520 -93.5075 -152.6405 -135 0])
end
rob.moveToJointTargetPositions([45 -125 -100 -135 -135 0]);
while ~rob.checkPoseReached([45 -125 -100 -135 -135 0])
end
pause(0.5)
rob.moveToJointTargetPositions([45 -113.8520 -93.5075 -152.6405 -135 0]);
while ~rob.checkPoseReached([45 -113.8520 -93.5075 -152.6405 -135 0])
end
rob.moveToJointTargetPositions([-25 -113.8520 -93.5075 -152.6405 -25 0]);
while ~rob.checkPoseReached([-25 -113.8520 -93.5075 -152.6405 -25 0])
end
rob.moveToJointTargetPositions([-25 -125 -100 -135 -25 0]);
while ~rob.checkPoseReached([-25 -125 -100 -135 -25 0])
end
pause(0.5)
rob.moveToJointTargetPositions([-25 -113.8520 -93.5075 -152.6405 -25 0]);
while ~rob.checkPoseReached([-25 -113.8520 -93.5075 -152.6405 -25 0])
end
rob.goHome();
while ~rob.checkPoseReached(rob.homeJointTargetPositions)
end

%% Disconnect
rob.disconnect();
