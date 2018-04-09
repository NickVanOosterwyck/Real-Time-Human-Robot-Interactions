classdef kinectcore < handle
    %kinectcore Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected)
        cam                 % selected kinect class
        CameraLocation      % Location of camera [x y z alfa beta gamma] (eul: ZYX absolute axes/ XYZ own axes)
        homeCameraLocation  % Home location of camera
        detectionVol        % Dimensions of volume where object are detected
        worktableVol        % Dimensions of worktable 
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
            RotMat = eul2rotm(obj.CameraLocation(4:6)./180.*pi,'XYZ');
            HomoTransMat = [ RotMat obj.CameraLocation(1:3).';...
                zeros(1,3) 1];
            XYZrow = ptCloud.Location.';
            XYZ = [XYZrow;ones(1,length(XYZrow))];
            Result = HomoTransMat*XYZ;
            ptCloud = pointCloud(Result(1:3,:).');
        end
        


        function plotPointCloud(obj,ptCloud)
            pcshow(ptCloud,'MarkerSize',8);
            axis equal
            axis(obj.detectionVol)
            xlabel('X [m]');
            ylabel('Y [m]');
            zlabel('Z [m]');
            hold on
            quiver3(0,0,0,1,0,0,0.3,'r','Linewidth',1.5)
            quiver3(0,0,0,0,1,0,0.3,'g','Linewidth',1.5)
            quiver3(0,0,0,0,0,1,0.3,'b','Linewidth',1.5)
            plotCamera('Location',obj.CameraLocation(1:3),'Orientation',eul2rotm(obj.CameraLocation(4:6)./180.*pi,'XYZ').','Opacity',0,'Size',0.1);
            hold off
        end
        function getPointCloudComparison(obj)
            ptCloudDesampled = obj.getPointCloud('Desampled');
            ptCloudFiltered = obj.getPointCloud('Filtered',ptCloudDesampled);
            [dist,Point] = obj.calculateClosestPoint(ptCloudFiltered);
            
            figure('Name','PointCloud Comparison');
            subplot(1,2,1);
            title('PointCloud Desampled')
            obj.plotPointCloud(ptCloudDesampled);

            subplot(1,2,2);
            title('PointCloud Filtered')
            obj.plotPointCloud(ptCloudFiltered);
            hold on
            obj.drawRobotBase();
            obj.drawBox(obj.worktableVol);
            if ~isinf(dist)
                plot3([0 Point(1)],[0 Point(2)],[0.988 Point(3)],'r')
                plot3(Point(1),Point(2),Point(3),'r','Marker','o','LineWidth',2)
                text(Point(1)/2,Point(2)/2,((Point(3)-0.988)/2)+0.988,[' ' num2str(round(dist,2)) ' m'])
            else
                text(0,0,1.3,'No point detected')
            end
            hold off
        end
        function getPointCloudCalibration(obj)
            ptCloudRaw = obj.getPointCloud('Raw');
            figure('Name','PointCloud Calibration');
            obj.plotPointCloud(ptCloudRaw);
            hold on
            obj.drawRobotBase();
            obj.drawBox(obj.worktableVol);
            hold off
        end
        function [bodies]=getSkeleton(obj)
            [bodies]= obj.cam.getSkeleton();
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

