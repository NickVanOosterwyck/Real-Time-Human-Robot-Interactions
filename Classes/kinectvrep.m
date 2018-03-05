classdef kinectvrep < kinectcore & VREP_Projector
    %kinectvrep Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected)    
    end
    
    methods
        function obj = kinectvrep()
        end % conctructor

        function connect(obj)
            obj.Open('Kinect_sensor');clc;
            if (obj.clientID>-1)
                disp('Connected to remote API server! (Kinect)');
            else
                error('Problem with connection!!!\n%s','Make sure the simulation in VREP is running and try again.')
            end
            load cameraParams.mat cameraParams;
            I = obj.cameraParameters2IntrinsicMatrix(cameraParams);
            obj.setParams(copy(I));
            obj.moveHome();
        end
        function disconnect (obj)
            obj.Close();
        end
        function moveToCameraLocation(obj,Location)
            obj.CameraLocation = Location;
            [~]=obj.simObj.simxSetObjectOrientation(obj.clientID,obj.handle,-1,obj.CameraLocation(4:6)./180.*pi,obj.simObj.simx_opmode_oneshot);
            [~]=obj.simObj.simxSetObjectPosition(obj.clientID,obj.handle,-1,obj.CameraLocation(1:3),obj.simObj.simx_opmode_oneshot);
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

