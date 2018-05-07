classdef kinectcore < handle
    %kinectcore Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected)
        cam                 % selected kinect class
        CameraLocation      % Location of camera [x y z alfa beta gamma] (eul: ZYX absolute axes/ XYZ own axes)
        homeCameraLocation  % Home location of camera
        detectionVol        % Dimensions of volume where object are detected
        worktableVol        % Dimensions of worktable
        SkeletonConnectionMap %
    end
    
    methods
        function obj = kinectcore(CameraType)
            p = inputParser;
            acceptedInput = {'vrep','real'};
            p.addRequired('CameraType',@(x) any(validatestring(x,acceptedInput)));
            p.parse(CameraType);
            if p.Results.CameraType == 'vrep'
                obj.cam = kinectvrep();
            else
                obj.cam = kinectreal();
            end
            obj.homeCameraLocation = [0.67 1.68 1.09 90 0 0];
            obj.CameraLocation = zeros(1,6);
            obj.detectionVol = [-2 1.5 -2 1.6 0 2.3];
            obj.worktableVol = [-0.08 1.42 -0.7 0.7 0 0.845];
            obj.SkeletonConnectionMap = [ [4 3];  % Spine
                          [3 21];
                          [21 2];
                          [2 1];
                          [21 9];   % Right Arm
                          [9 10]; 
                          [10 11];
                          [11 12];
                          [12 24];
                          [12 25];
                          [21 5];  % Left Arm
                          [5 6];
                          [6 7];
                          [7 8];
                          [8 22];
                          [8 23];
                          [1 17];   % Right Leg
                          [17 18];
                          [18 19];
                          [19 20];
                          [1 13];   % Left Leg
                          [13 14];
                          [14 15];
                          [15 16];
                        ];
        end % constructor
        function set.CameraLocation(obj,Location)
            if length(Location)==6 && isnumeric(Location) ...
                    && sum(Location(4:6)>180)==0 && sum(Location(4:6)<-180)==0
                obj.CameraLocation = Location;
                if sum(Location(1:3)>10)>0 || sum(Location(1:3)<-10)>0
                    warning('Camera is placed more than 10m away from WCS.')
                end
            else
                error('Invalid CameraLocation!\n%s',...
                    'Values are outside range.')
            end
        end
        function set.detectionVol(obj,Vol)
            if length(Vol)==6 && isnumeric(Vol)
                obj.detectionVol =Vol;
            else
                error('Invalid detection volume!\n%s',...
                    'Values are invalid.')
            end
        end
        function set.worktableVol(obj,Vol)
            if length(Vol)==6 && isnumeric(Vol)
                obj.worktableVol =Vol;
            else
                error('Invalid worktable volume!\n%s',...
                    'Values are invalid.')
            end
        end
        
        function connect(obj)
            obj.cam.connectDif();
            obj.moveToCameraLocation(obj.homeCameraLocation);
        end
        function disconnect (obj)
            obj.cam.Close();
        end
        function moveToCameraLocation(obj,Location)
            obj.CameraLocation = Location;
            obj.cam.moveToCameraLocationDif(Location);
        end
        function setdetecionVol(obj,detectionVol)
            obj.detectionVol=detectionVol;
        end
        function [ptCloud] = getPointCloud(obj,varargin)
            % input
            p = inputParser;
            acceptedInput = {'Raw','Desampled','Filtered'};
            p.addRequired('obj');
            p.addOptional('Type','Raw',@(x) any(validatestring(x,acceptedInput)));
            p.addOptional('ptCloudIn',[]);
            p.parse(obj,varargin{:});
            
            % get new pointcloud if no pointcloud is provided
            if isempty(p.Results.ptCloudIn)
                XYZ = obj.cam.GetFrame(TofFrameType.XYZ_3_COLUMNS);
                ptCloud = pointCloud(XYZ);
            else
                ptCloud = ptCloudIn;
            end
            
            if any(strcmp(p.Results.Type,'Raw'))
                ptCloud = obj.transformPointCloud(ptCloud); % transform to WCS
                ptCloud = obj.selectBox(ptCloud,obj.detectionVol,0); % select detection area
            end
            
            
            if any(strcmp(p.Results.Type,{'Desampled','Filtered'}))
                [ptCloud,~] = removeInvalidPoints(ptCloud);
                ptCloud = pcdownsample(ptCloud,'gridAverage',0.05);
                if isa(obj.cam,'kinectvrep')
                    ptCloud = obj.selectBox(ptCloud,[-inf inf -inf inf -inf obj.cam.ClippingPlane],0.05);
                end
                ptCloud = obj.transformPointCloud(ptCloud); % transform to WCS
                ptCloud = obj.selectBox(ptCloud,obj.detectionVol,0); % select detection area
            end
            
            if any(strcmp(p.Results.Type,{'Filtered'}))
                ptCloud = obj.removeBox(ptCloud,[1 1.5 0 0.7 0 2.3],0.1); % remove computer
                ptCloud = pcdenoise(ptCloud,'Threshold',0.01,'NumNeighbors',20);
                ptCloud = pcdenoise(ptCloud,'Threshold',0.1);
                box=obj.worktableVol;
                box(6) = 2.3;
                ptCloud = obj.removeBox(ptCloud,box,0.1); % remove worktable
            end
            
        end
        function [RGB] = getRGB(obj)
            RGB = obj.cam.GetFrame(TofFrameType.RGB_IMAGE);
        end
        function [ptCloud] = transformPointCloud(obj,ptCloud)
            RotMat = eul2rotm(obj.CameraLocation(4:6)/180*pi,'XYZ');
            HomoTransMat = [ RotMat obj.CameraLocation(1:3).';...
                zeros(1,3) 1];
            XYZrow = ptCloud.Location.';
            XYZ = [XYZrow;ones(1,length(XYZrow))];
            Result = HomoTransMat*XYZ;
            ptCloud = pointCloud(Result(1:3,:).');
        end
        function [bodies,varargout]=getSkeleton(obj)
            [bodies]= obj.cam.getSkeleton();
            n=length(bodies);
            varargout{1}=n;
            if ~isempty(bodies)
                RotMat = eul2rotm(obj.CameraLocation(4:6)/180*pi,'XYZ');
                HomoTransMat = [ RotMat obj.CameraLocation(1:3).';...
                    zeros(1,3) 1];
                for i=1:n
                    XYZ3 = bodies(i).Position;
                    XYZ4 = [XYZ3;ones(1,length(XYZ3))];
                    Result = HomoTransMat*XYZ4;
                    bodies(i).Position= Result(1:3,:);
                end
            end
        end
        function [h] = getHandHeight(obj,varargin)
            p = inputParser;
            p.StructExpand = false;
            acceptedMode = {'ptCloud','Skeleton'};
            acceptedSide = {'Left','Right'};
            acceptedOut = {'All','Max'};
            p.addRequired('obj');
            p.addRequired('Mode',@(x) any(validatestring(x,acceptedMode)));
            p.addOptional('Side','Right',@(x) any(validatestring(x,acceptedSide)));
            p.addOptional('Out','Max',@(x) any(validatestring(x,acceptedOut)));
            p.addOptional('dataIn',[]);
            p.parse(obj,varargin{:});
            
            % get data if necessary
            if strcmp(p.Results.Mode,'ptCloud')
                % get new pointcloud if no pointcloud is provided
                if isempty(p.Results.dataIn)
                    ptCloud = obj.getPointCloud('Filtered');
                else
                    ptCloud = p.Results.dataIn;
                end
            elseif strcmp(p.Results.Mode,'Skeleton')
                % get new bodies if no bodies is provided
                if isempty(p.Results.dataIn)
                    bodies = obj.getSkeleton();
                else
                    bodies = p.Results.dataIn;
                end
            end
            
            % set indice for right/left hand
            if strcmp(p.Results.Side,'Right')
                i=24;
            else
                i=22;
            end
            
            % get height of hand
            if strcmp(p.Results.Mode,'ptCloud')
                if ptCloud.Count ~=0
                    h = ptCloud.ZLimits(2);
                else
                    h=0;
                end
            elseif strcmp(p.Results.Mode,'Skeleton')
                h=zeros(1,6);
                if ~isempty(bodies)
                    numBodies=length(bodies);
                    for j=1:numBodies
                        if bodies(j).TrackingState(i)==2
                            h(j)=bodies(j).Position(3,i);
                        else
                            h(j)=0;
                        end
                    end
                else
                    h=0;
                end
            end
            if strcmp(p.Results.Out,'Max')
                h=max(h);
            end
            
        end
        function [Position] = getHandPosition(obj,varargin)
            p = inputParser;
            p.StructExpand = false;
            acceptedSide = {'Left','Right'};
            acceptedCS = {'World','Robot'};
            p.addRequired('obj');
            p.addOptional('Side','Right',@(x) any(validatestring(x,acceptedSide)));
            p.addOptional('CS','World',@(x) any(validatestring(x,acceptedCS)));
            p.addOptional('dataIn',[]);
            p.parse(obj,varargin{:});
            
            % get new bodies if no bodies is provided
            if isempty(p.Results.dataIn)
                bodies = obj.getSkeleton();
            else
                bodies = p.Results.dataIn;
            end
            
            % set indice for right/left hand
            if strcmp(p.Results.Side,'Right')
                i=24;
            else
                i=22;
            end
            
            Position=zeros(1,3);
            if ~isempty(bodies)
                if bodies(1).TrackingState(i)==2
                    for k=1:3
                        Position(k)=bodies(1).Position(k,i);
                    end
                else
                    Position=[Inf Inf Inf];
                end
            else
                Position=[Inf Inf Inf];
            end
            
            % transform to robot coordinate system
            if strcmp(p.Results.CS,'Robot')
                Position = Position*1000;
                Position(3)=Position(3)-860;
            end
            
            
        end
        function [varargout] = drawSkeleton(obj,varargin)
            p=inputParser;
            p.StructExpand = false;
            p.addRequired('obj');
            p.addOptional('bodies',struct([]));
            p.parse(obj,varargin{:});
            
            % get new bodies if no bodies is provided
            if isempty(p.Results.bodies)
                bodies = obj.getSkeleton();
            else
                bodies = p.Results.bodies;
            end
            
            
            % Marker colors for up to 6 bodies.
            colors = ['r';'g';'b';'c';'y';'m'];
            n=0;
            numBodies=length(bodies);
            if ~isempty(bodies)
                for j=1:numBodies
                    pos=bodies(j).Position;
                    for i = 1:24
                        if bodies(j).TrackingState(obj.SkeletonConnectionMap(i,1))==2 && bodies(j).TrackingState(obj.SkeletonConnectionMap(i,2))==2
                            X=[pos(1,obj.SkeletonConnectionMap(i,1)) pos(1,obj.SkeletonConnectionMap(i,2))];
                            Y=[pos(2,obj.SkeletonConnectionMap(i,1)) pos(2,obj.SkeletonConnectionMap(i,2))];
                            Z=[pos(3,obj.SkeletonConnectionMap(i,1)) pos(3,obj.SkeletonConnectionMap(i,2))];
                            plot3(X,Y,Z, 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '+', 'Color', colors(j));
                            n=n+1;
                        end
%                         if bodies(j).TrackingState(obj.SkeletonConnectionMap(i,1))==1 || bodies(j).TrackingState(obj.SkeletonConnectionMap(i,2))==1
%                             X=[pos(1,obj.SkeletonConnectionMap(i,1)) pos(1,obj.SkeletonConnectionMap(i,2))];
%                             Y=[pos(2,obj.SkeletonConnectionMap(i,1)) pos(2,obj.SkeletonConnectionMap(i,2))];
%                             Z=[pos(3,obj.SkeletonConnectionMap(i,1)) pos(3,obj.SkeletonConnectionMap(i,2))];
%                             plot3(X,Y,Z, 'LineWidth', 1.5, 'LineStyle', '--', 'Marker', '+', 'Color', colors(j));
%                             n=n+1;
%                         end
                    end
                end
            end
            varargout{1}=n;
        end
        function [varargout] = createAxis(obj,varargin)
            p=inputParser;
            acceptedAxis = {'auto','detVol'};
            p.addRequired('obj');
            p.addOptional('Axis','auto',@(x) any(validatestring(x,acceptedAxis)));
            p.parse(obj,varargin{:});
            
            f=figure;
            ax=axes;
            axis equal
            if strcmp(p.Results.Axis,'auto')
                axis auto
            elseif strcmp(p.Results.Axis,'detVol')
                axis(obj.detectionVol)
            end
            xlabel('X [m]');
            ylabel('Y [m]');
            zlabel('Z [m]');
            hold on
            quiver3(0,0,0,1,0,0,0.3,'r','Linewidth',1.5)
            quiver3(0,0,0,0,1,0,0.3,'g','Linewidth',1.5)
            quiver3(0,0,0,0,0,1,0.3,'b','Linewidth',1.5)
            plotCamera('Location',obj.CameraLocation(1:3),'Orientation',eul2rotm(obj.CameraLocation(4:6)./180.*pi,'XYZ').','Opacity',0,'Size',0.1);
            grid on
            view(3)
            varargout{1}=f;
            varargout{2}=ax;
            
        end
  

    end
    
    methods (Static)
        function [ptCloud] = selectBox(ptCloud,dim,off)
            ROI = [dim(1)+off dim(2)-off dim(3)+off dim(4)-off dim(5)+off dim(6)-off];
            indices = findPointsInROI(ptCloud,ROI);
            [ptCloud] = select(ptCloud,indices);
        end
        function [ptCloud] = removeBox(ptCloud,dim,off)
            ROI = [dim(1)-off dim(2)+off dim(3)-off dim(4)+off dim(5)-off dim(6)+off];
            ind = findPointsInROI(ptCloud,ROI);
            indInv = setdiff((1:ptCloud.Count),ind);
            [ptCloud] = select(ptCloud,indInv);
            
        end 
        function drawRobotBase()
            % add robot base cylinder
            [x1,y1,z1] = cylinder(0.17/2,10);
            z1(1, :) = 0.845;
            z1(2, :) = 0.845+0.205;
            mesh(x1,y1,z1,'FaceAlpha',0,'EdgeColor','b')
        end
        function drawBox(dim)
            x2 = [dim(1) dim(1) dim(1) dim(1) dim(1); dim(2) dim(2) dim(2) dim(2) dim(2)];
            y2 = [dim(3) dim(3) dim(4) dim(4) dim(3); dim(3) dim(3) dim(4) dim(4) dim(3)];
            z2 = [dim(6) dim(5) dim(5) dim(6) dim(6); dim(6) dim(5) dim(5) dim(6) dim(6)];
            mesh(x2,y2,z2,'FaceAlpha',0,'EdgeColor','b');
        end
        
    end
    
    
end

