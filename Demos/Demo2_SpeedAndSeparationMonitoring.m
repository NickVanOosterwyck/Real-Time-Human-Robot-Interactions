%% Add path
addpath(genpath(pwd)); % make sure current directory is the top map!

%% Clear
clear; close all; clc

%% Create & Connect
CameraType = 'vrep';    % vrep or real
RobotType = 'real';     % vrep or real

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
rSlow = 1.7;

%% Go home
% limit speed
ctrl.rob.goHome(0.2);
while ~ctrl.rob.checkPoseReached(ctrl.rob.homeJointTargetPositions,0.2)
end
disp('Robot is ready in home pose.')

%% Cycle
MaxSpeedFactor = 0.2;
Range = 0.2;
treshold = 0.4;
iterations = 1;
Ref = 'Base'; % choose TCP or Base

state = 0;
LastDist=Inf;
PrevDist =[Inf Inf Inf Inf Inf];
dis = controlDisplay();
dis.setValues('Reference',Ref);
for it = 1:iterations
    i = 1;
    for i = 1:length(Path)
        state=1;
        while ~ctrl.rob.checkPoseReached(Path(i,:),Range)
            %tic
            [Dist,~] = ctrl.getClosestPoint(Ref);
            PrevDist= [PrevDist(2:5) Dist];
            Dist = mean(PrevDist);
            if  Dist<rStop
                if state ~=0 && abs(LastDist-Dist)>treshold
                    LastDist = Dist;
                    ctrl.rob.stopRobot(); disp('Robot is stopped')
                    state=0;
                end
            elseif Dist>rStop && Dist<rSlow
                if abs(LastDist-Dist)>treshold || state==1
                    LastDist = Dist;
                    Speedfactor = min(((Dist-rStop)/(rSlow-rStop))*MaxSpeedFactor,MaxSpeedFactor);
                    ctrl.rob.stopRobot();
                    ctrl.rob.moveToJointTargetPositions(Path(i,:),Speedfactor);
                    state=3; disp(['Robot is moved slower to Pose' num2str(i) ' with speedfactor ' num2str(Speedfactor)])
                    dis.setValues('Speedfactor',Speedfactor);
                end
            else
                if state ~=2 && (abs(LastDist-Dist)>treshold || state==1)
                    LastDist = Dist;
                    ctrl.rob.moveToJointTargetPositions(Path(i,:),MaxSpeedFactor);
                    state=2; disp(['Robot is moved normal to Pose' num2str(i)])
                    dis.setValues('Speedfactor',MaxSpeedFactor);
                end
            end
            dis.setValues('Dist',Dist,'LastDist',LastDist,'TargetPose',i,'State',state);
            %toc
        end
    end
end
close all
disp('End of loop reached')

%% States
% 0	Stop
% 1	Next Target
% 2	Move normal
% 3	Move slow

