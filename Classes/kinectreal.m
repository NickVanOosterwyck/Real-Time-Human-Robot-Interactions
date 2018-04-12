classdef kinectreal < handle & Kinect
    %kinectreal Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected)
        
    end
    
    methods
        function obj = kinectreal()
            
        end % constructor
        
        function connectDif(obj)
            obj.Open('body');
            %load camera_parameters.mat Ip;
            %obj.setParams(copy(Ip));
        end
        function moveToCameraLocationDif(~,~)
            disp('Camera Position has been changed.');
        end
        function [bodies]= getSkeleton(obj)
            obj.GetFrame(TofFrameType.DEPTH_IMAGE);
            [bodies, ~, ~] = obj.CameraProtocol.CameraSettings.getBodies('Euler');
        end

    end
end



