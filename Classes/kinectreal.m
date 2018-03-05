classdef kinectreal < kinectcore & Kinect
    %kinectreal Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected)
    end
    
    methods
        function obj = kinectreal()
        end % constructor
        
        function connect(obj)
            obj.Open();
            load camera_parameters.mat Ip;
            %obj.setParams(copy(Ip));
            obj.moveHome();
        end
        function disconnect (obj)
            obj.Close();
        end
        function moveToCameraLocation(obj,Location)
            obj.CameraLocation = Location;
        end
        function moveHome(obj)
            obj.moveToCameraLocation(obj.homeCameraLocation);
        end
        function [ptCloud] = getRawPointCloud(obj)
            XYZ = obj.GetFrame(TofFrameType.XYZ_3_COLUMNS);
            ptCloud = pointCloud(XYZ);
            ptCloud = obj.transformPointCloud(ptCloud);
        end
        function [ptCloud] = getDesampledPointCloud(obj)
            XYZ = obj.GetFrame(TofFrameType.XYZ_3_COLUMNS);
            ptCloud = pointCloud(XYZ);
            ptCloud = obj.desamplePointCloud(ptCloud);
            ptCloud = obj.transformPointCloud(ptCloud);
        end
        function [ptCloud] = getFilteredPointCloud(obj)
            XYZ = obj.GetFrame(TofFrameType.XYZ_3_COLUMNS);
            ptCloud = pointCloud(XYZ);
            ptCloud = obj.desamplePointCloud(ptCloud);
            ptCloud = obj.removeClippingPlane(ptCloud,5);
            ptCloud = obj.transformPointCloud(ptCloud);
            ptCloud = obj.selectBox(ptCloud,obj.detectionVol,0.1); % select detection area
            ptCloud = obj.removeBox(ptCloud,obj.worktableVol,0.1); % remove worktable
        end
        function [RGB] = getRGB(obj)
            RGB = obj.GetFrame(TofFrameType.RGB_IMAGE);
        end
    end
end



