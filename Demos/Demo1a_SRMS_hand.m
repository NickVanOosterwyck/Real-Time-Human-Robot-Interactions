%% Add path
addpath(genpath(pwd)); % make sure current directory is the top map!

%% Clear
clear; close all; clc

%% Create & Connect
CameraType = 'vrep';    % vrep or real
RobotType = 'vrep';     % vrep or real

ctrl = controller(CameraType,RobotType);
ctrl.connect();

%% Path
%-- set positions
Home = ctrl.rob.homeJointTargetPositions;
PickUp = [45 -110 -80 -170 -135 0];
PickUpApp = [45 -113.2953  -44.7716 -201.9331 -135 0];
Place = [-25 -110 -80 -170 -25 0];
PlaceApp = [-25 -113.2953  -44.7716 -201.9331 -25 0];

%-- create path
Path =[Home;PickUpApp;PickUp;PickUpApp;PlaceApp;Place;PlaceApp;Home];

%% Go home
ctrl.rob.goHome(true);
disp('Robot is ready in home pose.')

%% Demo 1: Safety Rated Monitored Stop
rStop = 1;
iterations = 1;
Mode = 'ptCloud'; % choose Skeleton or ptCloud
a=5; v=0.2; t=0; r=0;

for it = 1:iterations
    for i = 1:size(Path,1)
        flag = 0;
        while ~ctrl.rob.checkPoseReached(Path(i,:),0.05)
            if strcmp(Mode,'Skeleton')
                data=ctrl.cam.getSkeleton();
            elseif strcmp(Mode,'ptCloud')
                data = ctrl.cam.getPointCloud('Filtered');
            end
            [Dist,~,~] = ctrl.getClosestPoint(Mode,'TCP',data);
             
            if Dist<rStop
                ctrl.rob.stopj(10);
                flag=0;
            elseif Dist>rStop
                if flag == 0
                    ctrl.rob.movej(Path(i,:),a,v,t,r);
                    flag=1;
                end
            end
            
        end
    end
end
disp('End of loop reached')

