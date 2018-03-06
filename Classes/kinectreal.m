classdef kinectreal < handle
    %kinectreal Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected)

    end
    
    methods
        function obj = kinectreal()
        end % constructor
        
        function connectDif(obj)
            obj.Open();
            %load camera_parameters.mat Ip;
            %obj.setParams(copy(Ip));
        end
        function moveToCameraLocationDif(~,Location)
            disp(['Camera Position is ',num2str(Location),' (x y z alfa beta gamma)']);
        end

        
        
    end
end



