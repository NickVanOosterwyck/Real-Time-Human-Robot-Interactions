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
        
        function [Dist,StartPoint,EndPoint] = getClosestPoint(obj,Mode,Reference,varargin)
        %getClosestPoint calculates the closest distance between the
        %reference and a point in a pointcloud. The function will get a new
        %pointcloud if no pointcloud is provided. The reference kan be the
        %robot base or TCP.
            p = inputParser;
            p.StructExpand = false;
            acceptedMode = {'ptCloud','Skeleton'};
            acceptedRef = {'Base','TCP'};
            p.addRequired('obj');
            p.addRequired('Mode',@(x) any(validatestring(x,acceptedMode)));
            p.addRequired('Reference',@(x) any(validatestring(x,acceptedRef)));
            p.addOptional('dataIn',[]);
            p.parse(obj,Mode,Reference,varargin{:});
            
            % get data if necessary
            if strcmp(p.Results.Mode,'ptCloud')
                % get new pointcloud if no pointcloud is provided
                if isempty(p.Results.dataIn)
                    ptCloud = obj.cam.getPointCloud('Filtered');
                else
                    ptCloud = p.Results.dataIn;
                end
            elseif strcmp(p.Results.Mode,'Skeleton')
                % get new bodies if no bodies is provided
                if isempty(p.Results.dataIn)
                    bodies = obj.cam.getSkeleton();
                else
                    bodies = p.Results.dataIn;
                end
            end
            
            % set startpoint
            if strcmp(p.Results.Reference,'Base')
                StartPoint = [0 0 0.988];
            elseif strcmp(p.Results.Reference,'TCP')
                JointPositions = obj.rob.getJointPositions();
                [StartPoint,~,~] = obj.rob.ForwKin(JointPositions);
                StartPoint =StartPoint(1:3)./1000;
                StartPoint(3)=StartPoint(3)+0.86;
            end
            
            % calculate distances
            if strcmp(p.Results.Mode,'ptCloud')
                [indices, dists] = findNearestNeighbors(ptCloud,StartPoint,11,'Sort',true);
                if ~isempty(indices)&&length(indices)>10
                    Dist = dists(10);
                    EndPoint = ptCloud.Location(indices(10),:);
                else
                    Dist = inf;
                    EndPoint = [inf,inf,inf];
                end
            elseif strcmp(p.Results.Mode,'Skeleton')
                if ~isempty(bodies)
                    numBodies=length(bodies);
                    distances = zeros(numBodies,25);
                    for j=1:numBodies
                        pos=bodies(j).Position;
                        for i=1:25
                            if bodies(j).TrackingState(i) == 2
                                distances(j,i) = sqrt(((pos(1,i)-StartPoint(1))^2)+((pos(2,i)-StartPoint(2))^2)+((pos(3,i)-StartPoint(3))^2));
                            else
                                distances(j,i) = inf;
                            end
                        end
                    end
                    if numBodies>1
                        [Min,indy]=min(distances);
                        [Dist,indx]=min(Min);
                        indb=indy(indx);
                    else
                        [Dist,indx]=min(distances);
                        indb=1;
                    end
                    pos=bodies(indb).Position;
                    EndPoint = [pos(1,indx) pos(2,indx) pos(3,indx)];
                else
                    Dist = inf;
                    EndPoint = [inf,inf,inf];
                end
            end
        end
%         function showPlayer(obj)
%             player = pcplayer(obj.cam.detectionVol(1:2),obj.cam.detectionVol(3:4),obj.cam.detectionVol(5:6),'MarkerSize',8);
%             while isOpen(player)
%                 tic
%                 ptCloud = obj.cam.getPointCloud('Filtered');
%                 view(player, ptCloud);
%                 toc
%             end
%             clc
%         end %DEPRECATED
        function showTrackingPlayer(obj,Mode,Reference,varargin)
            p = inputParser;
            acceptedMode = {'ptCloud','Skeleton'};
            acceptedRef = {'Base','TCP'};
            p.addRequired('obj');
            p.addRequired('Mode',@(x) any(validatestring(x,acceptedMode)));
            p.addRequired('Reference',@(x) any(validatestring(x,acceptedRef)));
            p.addOptional('Robot',true,@islogical);
            p.parse(obj,Mode,Reference,varargin{:});
            
            obj.cam.createAxis();
            title('Tracking Player')
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
                % draw data
                if strcmp(p.Results.Mode,'ptCloud')
                    ptCloud = obj.cam.getPointCloud('Filtered');
                    pcshow(ptCloud,'MarkerSize',8)
                    axis(obj.cam.detectionVol)
                    [dist,StartPoint,EndPoint] = obj.getClosestPoint('ptCloud',p.Results.Reference,ptCloud);
                elseif strcmp(p.Results.Mode,'Skeleton')
                    bodies = obj.cam.getSkeleton;
                    nSkel=obj.cam.drawSkeleton(bodies);
                    axis([-2 3 -3 1.8 0 2])
                    [dist,StartPoint,EndPoint] = obj.getClosestPoint('Skeleton',p.Results.Reference,bodies);
                end
                
                % draw robot
                if p.Results.Robot
                    obj.rob.drawRobot();
                    nRob=7;
                else
                    nRob=0;
                end
                
                % draw line with closest distance
                if ~isinf(dist)
                    plot3([EndPoint(1) StartPoint(1)],[EndPoint(2) StartPoint(2)],[EndPoint(3) StartPoint(3)],...
                        'k','LineStyle','--','Marker','o','LineWidth',1)
                    text(0,0,1.3,[' ' num2str(round(dist,2)) ' m'])
                else
                    plot3([0 0],[0 0],[0 0])
                    text(0,0,1.3,'No point detected')
                end
                
                % delete previous plots
                drawnow
                children = get(gca, 'children');
                delete(children(1:2+nRob));
                if strcmp(p.Results.Mode,'ptCloud')
                    delete(children(3+nRob));
                elseif strcmp(p.Results.Mode,'Skeleton')
                    if nSkel>0
                        delete(children(3+nRob:2+nRob+nSkel));
                    end
                end
                toc
            end
            close
            clc
            
        end
        function showDistanceCalculation(obj,Mode,Reference)
            p = inputParser;
            acceptedMode = {'ptCloud','Skeleton'};
            acceptedRef = {'Base','TCP'};
            p.addRequired('obj');
            p.addRequired('Mode',@(x) any(validatestring(x,acceptedMode)));
            p.addRequired('Reference',@(x) any(validatestring(x,acceptedRef)));
            p.parse(obj,Mode,Reference);
            
            
            if strcmp(p.Results.Mode,'ptCloud')
                obj.cam.createAxis('detVol'); hold on
                ptCloud = obj.cam.getPointCloud('Filtered');
                pcshow(ptCloud,'MarkerSize',8);
                axis(obj.cam.detectionVol)
                [dist,StartPoint,EndPoint] = obj.getClosestPoint('ptCloud',p.Results.Reference,ptCloud);
            elseif strcmp(p.Results.Mode,'Skeleton')
                obj.cam.createAxis('auto'); hold on
                bodies = obj.cam.getSkeleton;
                obj.cam.drawSkeleton(bodies);
                [dist,StartPoint,EndPoint] = obj.getClosestPoint('Skeleton',p.Results.Reference,bodies);
            end
            
            obj.cam.drawRobotBase();
            obj.cam.drawBox(obj.cam.worktableVol);
            obj.rob.drawRobot();
            title('Distance Calculation')
            
            if ~isinf(dist)
                plot3([EndPoint(1) StartPoint(1)],[EndPoint(2) StartPoint(2)],[EndPoint(3) StartPoint(3)],'r','LineWidth',2)
                plot3(StartPoint(1),StartPoint(2),StartPoint(3),'r','Marker','o','LineWidth',2)
                plot3(EndPoint(1),EndPoint(2),EndPoint(3),'r','Marker','o','LineWidth',2)
                text(0,0,1.3,[' ' num2str(round(dist,2)) ' m'])
            else
                text(0,0,1.3,'No point detected')
            end
            hold off
            
        end
        
    end
end

