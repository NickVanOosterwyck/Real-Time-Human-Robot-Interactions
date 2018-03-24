classdef controller < handle %& kinectcore & ur10core
    %controller Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        cam     % kinect object
        rob     % ur10 object
    end
    
    methods
        function obj = controller(CameraType,RobotType)
            p = inputParser;
            acceptedInput = {'vrep','real'};
            p.addRequired('CameraType',@(x) any(validatestring(x,acceptedInput)));
            p.addRequired('RobotType',@(x) any(validatestring(x,acceptedInput)));
            p.parse(CameraType,RobotType);
            
            obj.cam = kinectcore(p.Results.CameraType);
            obj.rob = ur10core(p.Results.RobotType);
        end
        function connect(obj)
            obj.cam.connect();
            obj.rob.connect();
        end
        
        function [Dist,StartPoint,EndPoint] = getClosestPoint(obj,Reference,varargin)
        %getClosestPoint calculates the closest distance between the
        %reference and a point in a pointcloud. The function will get a new
        %pointcloud if no pointcloud is provided. The reference kan be the
        %robot base or TCP.
            p = inputParser;
            acceptedInput = {'Base','TCP'};
            p.addRequired('obj');
            p.addRequired('Reference',@(x) any(validatestring(x,acceptedInput)));
            p.addOptional('ptCloudIn',[]);
            p.parse(obj,Reference,varargin{:});
            
            % get new pointcloud if no pointcloud is provided
            if isempty(p.Results.ptCloudIn)
                ptCloud = obj.cam.getPointCloud('Filtered');
            else
                ptCloud = p.Results.ptCloudIn;
            end
            % calculate distance
            if strcmp(p.Results.Reference,'Base')
                StartPoint = [0 0 0.988];
                [indices, dists] = findNearestNeighbors(ptCloud,StartPoint,11,'Sort',true);
                if ~isempty(indices)&&length(indices)>10
                    Dist = dists(10);
                    EndPoint = ptCloud.Location(indices(10),:);
                else
                    Dist = inf;
                    EndPoint = [inf,inf,inf];
                end
                
            else
                JointPositions = obj.rob.getJointPositions();
                [StartPoint,~,~] = obj.rob.ForwKin(JointPositions);
                StartPoint =StartPoint(1:3)./1000;
                StartPoint(3)=StartPoint(3)+0.86;
                [indices, dists] = findNearestNeighbors(ptCloud,StartPoint,11,'Sort',true);
                if ~isempty(indices)&&length(indices)>10
                    Dist = dists(10);
                    EndPoint = ptCloud.Location(indices(10),:);
                else
                    Dist = inf;
                    EndPoint = [inf,inf,inf];
                end
                
            end

        end
        function showPlayer(obj)
            player = pcplayer(obj.cam.detectionVol(1:2),obj.cam.detectionVol(3:4),obj.cam.detectionVol(5:6),'MarkerSize',8);
            while isOpen(player)
                tic
                ptCloud = obj.cam.getPointCloud('Filtered');
                view(player, ptCloud);
                toc
            end
            clc
        end
        function showTrackingPlayer(obj,Reference)
            p = inputParser;
            acceptedInput = {'Base','TCP'};
            p.addRequired('obj');
            p.addRequired('Reference',@(x) any(validatestring(x,acceptedInput)));
            p.parse(obj,Reference);
            
            figure('Name','PointCloud Tracking Player');
            obj.cam.plotPointCloud([Inf Inf Inf]);
            title('PointCloud Tracking Player')
            hold on
            obj.cam.drawRobotBase();
            obj.cam.drawBox(obj.cam.worktableVol);
            
            flag = 1;
            function pushbutton(~,~)
                flag = 0;
            end
            [~] = uicontrol('Style', 'pushbutton', 'String', 'Stop',...
                'Position', [20 20 50 20],...
                'Callback', @pushbutton);
            
            while flag==1
                tic
                ptCloud = obj.cam.getPointCloud('Filtered');
                pcshow(ptCloud,'MarkerSize',8)
                axis(obj.cam.detectionVol)
                grid on
                [dist,StartPoint,EndPoint] = obj.getClosestPoint(p.Results.Reference,ptCloud);
                if ~isinf(dist)
                    plot3([EndPoint(1) StartPoint(1)],[EndPoint(2) StartPoint(2)],[EndPoint(3) StartPoint(3)],'r')
                    plot3(StartPoint(1),StartPoint(2),StartPoint(3),'r','Marker','o','LineWidth',2)
                    plot3(EndPoint(1),EndPoint(2),EndPoint(3),'r','Marker','o','LineWidth',2)
                    text(0,0,1.3,[' ' num2str(round(dist,2)) ' m'])
                else
                    plot3([0 0],[0 0],[0 0])
                    plot3(Inf,Inf,Inf)
                    plot3(Inf,Inf,Inf)
                    text(0,0,1.3,'No point detected')
                end
                drawnow
                children = get(gca, 'children');
                delete(children(1:5));
                toc
            end
            close
            clc
            
        end
        function showProcessedPointCloud(obj,Reference)
            p = inputParser;
            acceptedInput = {'Base','TCP'};
            p.addRequired('obj');
            p.addRequired('Reference',@(x) any(validatestring(x,acceptedInput)));
            p.parse(obj,Reference);
            
            ptCloud = obj.cam.getPointCloud('Filtered');
            [dist,StartPoint,EndPoint] = obj.getClosestPoint(p.Results.Reference,ptCloud);
            
            title('PointCloud Processed')
            obj.cam.plotPointCloud(ptCloud);
            hold on
            obj.cam.drawRobotBase();
            obj.cam.drawBox(obj.cam.worktableVol);
            obj.rob.drawRobot();
            
            if ~isinf(dist)
                plot3([EndPoint(1) StartPoint(1)],[EndPoint(2) StartPoint(2)],[EndPoint(3) StartPoint(3)],'r')
                plot3(StartPoint(1),StartPoint(2),StartPoint(3),'r','Marker','o','LineWidth',2)
                plot3(EndPoint(1),EndPoint(2),EndPoint(3),'r','Marker','o','LineWidth',2)
                text(0,0,1.3,[' ' num2str(round(dist,2)) ' m'])
            else
                text(0,0,1.3,'No point detected')
            end
            
        end
        
    end
end

