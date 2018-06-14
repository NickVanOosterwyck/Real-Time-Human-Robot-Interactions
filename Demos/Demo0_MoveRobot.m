%% Add path
addpath(genpath(pwd)); % make sure current directory is the top map!

%% Clear
clear; close all; clc

%% Connect
rob=ur10core('vrep');    % vrep or real
rob.connect();

%% 2nd robot
rob2=ur10core('vrep');
rob2.connect();

%% Set up
%-- set positions
Home = rob.homeJointTargetPositions;
PickUp = [45 -110 -80 -170 -135 0];
PickUpApp = [45 -113.2953  -44.7716 -201.9331 -135 0];
Place = [-25 -110 -80 -170 -25 0];
PlaceApp = [-25 -113.2953  -44.7716 -201.9331 -25 0];

%-- create path
Path =[Home;PickUpApp;PickUp;PickUpApp;Home];

%% Go home
%rob2.goHome();
rob.goHome(true);
disp('Robot is ready in home pose.')

%% Demo 0: MoveRobot
iterations = 1;
a=5; v=0.2; t=0; r=0;

flag = 0;
for it = 1:iterations
    for i = 1:size(Path,1)
        flag = 0;
        while ~rob.checkPoseReached(Path(i,:),0.2)
            if  flag == 0
                rob.movej(Path(i,:),a,v,t,r);
                %rob2.movej(Path(i,:),a,v,t,r);
                flag = 1;
            end
            %pause(0.1) % for recording
        end
    end
end
disp('End of loop reached')


