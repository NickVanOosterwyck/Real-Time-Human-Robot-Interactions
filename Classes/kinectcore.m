classdef kinectcore < handle
    %kinectcore Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected)
        CameraLocation      % location of camera [x y z alfa beta gamma] (eul: ZYX absolute axes/ XYZ own axes)
        homeCameraLocation  % Home location of camera
    end
    
    methods
        function obj = kinectcore()
            obj.homeCameraLocation = [0 2.5 1 90 0 0];
            obj.CameraLocation = zeros(1,6);   
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

        function [ptCloud] = transformPointCloud(obj,ptCloud)
            RotMat = eul2rotm(obj.CameraLocation(4:6)./180.*pi,'XYZ');
            HomoTransMat = [ RotMat obj.CameraLocation(1:3).';...
                zeros(1,3) 1];
            XYZrow = ptCloud.Location.';
            XYZ = [XYZrow;ones(1,length(XYZrow))];
            Result = HomoTransMat*XYZ;
            ptCloud = pointCloud(Result(1:3,:).');
        end
        function showPointCloud(obj,ptCloud)
            figure('Name','PointCloud');
            pcshow(ptCloud)
            axis equal
            title('PointCloud')
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

    end
    
    methods (Static)
        function [ptCloud] = desamplePointCloud(ptCloud)
            [ptCloud,~] = removeInvalidPoints(ptCloud);
            ptCloud = pcdownsample(ptCloud,'gridAverage',0.05);
            ptCloud = pcdenoise(ptCloud);
        end
        function [ptCloud] = removeFloor(ptCloud,off)
            XYZ = ptCloud.Location;
            k=1;
            indices = -1;
            for i = 1:length(XYZ)
                if XYZ(i,3)>0+off
                    indices(k) = i;
                    k = k+1;
                end
            end
            if indices(1)>-1
                ptCloud = select(ptCloud,indices);
            else
                ptCloud = pointCloud([NaN NaN NaN]);
            end
        end
        function [ptCloud] = removeBox(ptCloud,dim,off)
            XYZ = ptCloud.Location;
            box = [dim(1)-off dim(2)+off dim(3)-off dim(4)+off dim(5)-off dim(6)+off];
            k=1;
            indices = -1;
            for i = 1:length(XYZ)
                if XYZ(i,1)<box(1) ||  XYZ(i,1)>box(2) ||  XYZ(i,2)<box(3) ...
                        ||  XYZ(i,2)>box(4) ||  XYZ(i,3)<box(5) ||  XYZ(i,3)>box(6)
                    indices(k) = i;
                    k = k+1;
                end
            end
            if indices(1)>-1
                ptCloud = select(ptCloud,indices);
            else
                ptCloud = pointCloud([NaN NaN NaN]);
            end
        end
        function [ptCloud] = removeClippingPlane(ptCloud,dist)
            XYZ = ptCloud.Location;
            k=1;
            indices = -1;
            for i = 1:length(XYZ)
                if XYZ(i,3)<dist-0.1
                    indices(k) = i;
                    k = k+1;
                end
            end
            if indices(1)>-1
                ptCloud = select(ptCloud,indices);
            else
                ptCloud = pointCloud([NaN NaN NaN]);
            end
        end

    end
    
    
end

