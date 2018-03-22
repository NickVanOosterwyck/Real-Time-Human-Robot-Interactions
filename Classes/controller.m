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
                ptCloud = ptCloudIn;
            end
            
            if strcmp(p.Results.Reference,'Base')
                [Dist,EndPoint] = obj.cam.calculateClosestPoint(ptCloud);
                StartPoint = [0 0 0.988];
            else
                [Dist,EndPoint,StartPoint] = obj.calculateClosestPointToTCP(ptCloud);
                
            end
            
        end
        function showTrackingPlayerToTCP(obj)
            figure('Name','PointCloud Tracking Player To TCP');
            title('PointCloud Tracking Player')
            obj.cam.plotPointCloud([Inf Inf Inf]);
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
                ptCloud = obj.cam.getFilteredPointCloud();
                pcshow(ptCloud,'MarkerSize',8)
                axis(obj.cam.detectionVol)
                grid on
                [dist,Point,TCP] = obj.calculateClosestPointToTCP(ptCloud);
                if ~isinf(dist)
                    plot3([TCP(1) Point(1)],[TCP(2) Point(2)],[TCP(3) Point(3)],'r')
                    plot3(Point(1),Point(2),Point(3),'r','Marker','o','LineWidth',2)
                    plot3(TCP(1),TCP(2),TCP(3),'r','Marker','o','LineWidth',2)
                    text(0,0,1.3,[' ' num2str(round(dist,2)) ' m'])
                else
                    plot3([0 0],[0 0],[0 0])
                    plot3(Inf,Inf,Inf)
                    plot3(Inf,Inf,Inf)
                    text(0,0,1.3,'No point detected')
                end
                drawnow
                children = get(gca, 'children');
                delete(children(1));
                delete(children(2));
                delete(children(3));
                delete(children(4));
                delete(children(5));
                toc
            end
            close
            clc
            
        end
        function showProcessedPointCloud(obj)
            ptCloudFiltered = obj.cam.getFilteredPointCloud();
            [dist,Point] = obj.cam.calculateClosestPoint(ptCloudFiltered);
            
            title('PointCloud Filtered')
            obj.cam.plotPointCloud(ptCloudFiltered);
            hold on
            obj.cam.drawRobotBase();
            obj.cam.drawBox(obj.cam.worktableVol);
            obj.drawRobot();
            if ~isinf(dist)
                plot3([0 Point(1)],[0 Point(2)],[0.988 Point(3)],'r')
                plot3(Point(1),Point(2),Point(3),'r','Marker','o','LineWidth',2)
                text(Point(1)/2,Point(2)/2,((Point(3)-0.988)/2)+0.988,[' ' num2str(round(dist,2)) ' m'])
            else
                text(0,0,1.3,'No point detected')
            end
            
        end
        function drawRobot(obj)
            [~,~,T] = obj.rob.ForwKin(obj.rob.getJointPositions);
            plot3(0,0,0.86,'g','Marker','o','LineWidth',2)
            
            P = zeros(7,3);
            for i=1:7
            P(i,:) = [T(1,4,i)/1000 T(2,4,i)/1000 T(3,4,i)/1000+0.86];
            plot3(P(i,1),P(i,2),P(i,3),'g','Marker','o','LineWidth',2)
            end
            
            plot3([0 P(1,1)],[0 P(1,2)],[0.86 P(1,3)],'g')
            for i=2:7
                plot3([P(i-1,1) P(i,1)],[P(i-1,2) P(i,2)],[P(i-1,3) P(i,3)],'g')
            end
            
        end
        
        function [Dist,Point,TCP] = calculateClosestPointToTCP(obj,ptCloud)
            JointPositions = obj.rob.getJointPositions();
            [TCP,~,~] = obj.rob.ForwKin(JointPositions);
            TCP =TCP(1:3)./1000;
            TCP(3)=TCP(3)+0.86;
            [indices, dists] = findNearestNeighbors(ptCloud,TCP,11,'Sort',true);
            if ~isempty(indices)&&length(indices)>10
                Dist = dists(10);
                Point = ptCloud.Location(indices(10),:);
            else
                Dist = inf;
                Point = [inf,inf,inf];
            end
            
        end

    end
end

