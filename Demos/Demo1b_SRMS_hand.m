%% Add path
addpath(genpath(pwd)); % make sure current directory is the top map!

%% Clear
clear; close all; clc

%% Create & Connect
CameraType = 'vrep';    % vrep or real
RobotType = 'vrep';     % vrep or real

ctrl = controller(CameraType,RobotType);
ctrl.connect();

%% Set up
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
hStop = 1.8;

%% Go home
%rob2.goHome();
ctrl.rob.goHome(true);
disp('Robot is ready in home pose.')

%% Demo 1: Safety Rated Monitored Stop
Range = 0.05;
iterations = 1;
th_dist = 0.1;
th_h = 0.1;
Ref = 'Base'; % choose TCP or Base
Mode = 'ptCloud'; % choose Skeleton or ptCloud
a=10; v=0.2; t=0; r=0;

SF=0; state=3; LastDist=Inf; LastH=Inf;
dis = GUI('ControlPanel',true,'LiveGraphDist',true,'LiveGraphSpeed',true);
dis.setValues('Reference',Ref);
tic
for it = 1:iterations
    i = 1;
    for i = 1:length(Path)
        ctrl.rob.movej(Path(i,:),a,v,t,r);
        while ~ctrl.rob.checkPoseReached(Path(i,:),Range)
             % get data
            if strcmp(Mode,'Skeleton')
                data=ctrl.cam.getSkeleton();
            elseif strcmp(Mode,'ptCloud')
                data = ctrl.cam.getPointCloud('Filtered');
            end
            h=ctrl.cam.getHandHeight(Mode,'Right','Max',data);
            [Dist,~,~] = ctrl.getClosestPoint(Mode,Ref,data);
            
            % determine speed
            if Dist<rStop && abs(LastDist-Dist)>th_dist
                LastDist=Dist;
                SF=0;
            elseif h>hStop && abs(LastH-h)>th_h
                LastH=h;
                SF=0;
            elseif Dist>rStop && h<hStop
                SF=1;
            end
            
            % send and plot speed
            time=toc;
            ctrl.rob.setSpeedFactor(SF);
            TCPSpeed = ctrl.rob.getTCPSpeed();
            dis.setValues('Dist',Dist,'TargetPose',i,'LastDist',LastDist,...
                'Height',h,'SF',SF,'Time',time,'State',state);
        end
    end
end
disp('End of loop reached')

