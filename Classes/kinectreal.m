classdef kinectreal < handle & Kinect
    %kinectreal Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected)
        SkeletonConnectionMap
    end
    
    methods
        function obj = kinectreal()
            obj.SkeletonConnectionMap = [[1 2]; % Spine
                         [2 3];
                         [3 4];
                         [3 5]; %Left Hand
                         [5 6];
                         [6 7];
                         [7 8];
                         [3 9]; %Right Hand
                         [9 10];
                         [10 11];
                         [11 12];
                         [1 17]; % Right Leg
                         [17 18];
                         [18 19];
                         [19 20];
                         [1 13]; % Left Leg
                         [13 14];
                         [14 15];
                         [15 16]];
        end % constructor
        
        function connectDif(obj)
            obj.Open();
            %load camera_parameters.mat Ip;
            %obj.setParams(copy(Ip));
        end
        function moveToCameraLocationDif(~,~)
            disp('Camera Position has been changed.');
        end
        function [bodies]= getSkeleton(obj)
            obj.GetFrame(TofFrameType.DEPTH_IMAGE);
            [bodies, ~, ~] = cam.CameraProtocol.CameraSettings.getBodies('Euler');
        end

    end
end



