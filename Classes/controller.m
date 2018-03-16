classdef controller < handle %& kinectcore & ur10core
    %controller Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        cam     % kinect object
        rob     % ur10 object
    end
    
    methods
        function obj = controller(cam_in,rob_in)
            obj.cam = kinectcore(cam_in);
            obj.rob = ur10core(rob_in);
        end
        function connect(obj)
            obj.cam.connect();
            obj.rob.connect();
        end
        
        function [Dist,Point,TCP] = calculateClosestPointToTCP(obj,ptCloud)
            
            
        end
    end
end

