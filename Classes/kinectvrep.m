classdef kinectvrep < handle & VREP_Projector
    %kinectvrep Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected) 
        h_camera
        ClippingPlane   % distance of clipping plane in VREP
    end
    
    methods
        function obj = kinectvrep()
            obj.ClippingPlane = 4;
        end % conctructor

        function connectDif(obj)
            obj.Open('Kinect_sensor');clc;
            if (obj.clientID>-1)
                disp('Connected to remote API server! (Kinect)');
            else
                error('Problem with connection!!!\n%s','Make sure the simulation in VREP is running and try again.')
            end
            % set cameraparameters
            load CameraData.mat I_depth;
            obj.setParams(copy(I_depth));
%             load cameraParams.mat cameraParams; %cameraParams are the RGB camera parameters
%             I = obj.cameraParameters2IntrinsicMatrix(cameraParams);
%             obj.setParams(copy(I));
        end
        function moveToCameraLocationDif(obj,Location)
            [~]=obj.simObj.simxSetObjectOrientation(obj.clientID,obj.handle,-1,Location(4:6)./180.*pi,obj.simObj.simx_opmode_oneshot);
            [~]=obj.simObj.simxSetObjectPosition(obj.clientID,obj.handle,-1,Location(1:3),obj.simObj.simx_opmode_oneshot);
        end
        function getSkeleton(~)
            error('getSkeleton is not supported in VREP')
        end
        
    end
end

