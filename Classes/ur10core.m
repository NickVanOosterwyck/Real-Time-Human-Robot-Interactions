classdef ur10core < handle
    %ur10core Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected)
        DH                          % Denavit-Hartenberg parameters
        TCPoffset                   % Distance between TCP and robot end
        homeJointTargetPositions;   % Joint positions of home pose in degree
        JointTargetPositions        % Joint target postions in degree
        TCPTargetPositions          % Position of TCP [x y z alfa beta gamma] (eul: ZYX absolute axes/ XYZ own axes)
        MaxJointSpeedFactor;        % Robot speed factor
    end
    
    methods
        function obj = ur10core()
            obj.homeJointTargetPositions = [0 -90 -90 -180 -90 0];
            obj.TCPTargetPositions = zeros(1,6);
            obj.JointTargetPositions = zeros(1,6);
            obj.MaxJointSpeedFactor = 1;
            obj.TCPoffset = 0;
            obj.DH.JointOffset = [128 0 0 164 116 92 obj.TCPoffset];
            obj.DH.LinkLength = [0 -612 -572 0 0 0 0];
            obj.DH.LinkTwist = [90 0 0 90 -90 0 0];
        end % constructor
        function set.MaxJointSpeedFactor(obj,Speed)
            if isnumeric(Speed) && Speed<=1 && Speed>=0
                obj.MaxJointSpeedFactor = Speed;
            else
                error('Invalid MaxJointSpeedFactor!\n%s',...
                    'Use values between 0 and 1.')
            end
        end
        function set.JointTargetPositions(obj,Positions)
            % input control
            if length(Positions)==6 && isnumeric(Positions)...
                    && sum(Positions>360)==0 && sum(Positions<-360)==0
                obj.JointTargetPositions = Positions;
            else
                error('Invalid JointTargetPositions!\n%s',...
                    'Use 1x6 array and values between -360 and 360.')
            end
            
        end
        function set.TCPTargetPositions(obj,Positions)
            if length(Positions)==6 && isnumeric(Positions)...
                    && sum(Positions(1:2)>1184)==0 && sum(Positions(1:2)<-1184)==0 ...
                    && sum(Positions(4:6)>180)==0 && sum(Positions(4:6)<-180)==0 ...
                    && Positions(3)<=1428
                obj.TCPTargetPositions = Positions;
            else
                error('Invalid TCPTargetPositions!\n%s',...
                    'Values are outside range.')
            end
        end
        
        function TCP = ForwKin(obj,JointPositions)
            ang = [JointPositions 0];
            d = obj.DH.JointOffset;
            a = obj.DH.LinkLength;
            alf = obj.DH.LinkTwist;
            HomoMat = eye(4,4);
            
            for i=1:7
                A = [cosd(ang(i)) -sind(ang(i))*cosd(alf(i)) sind(ang(i))*sind(alf(i)) a(i)*cosd(ang(i)); ...
                    sind(ang(i)) cosd(ang(i))*cosd(alf(i)) -cosd(ang(i))*sind(alf(i)) a(i)*sind(ang(i)); ...
                    0 sind(alf(i)) cosd(alf(i)) d(i); ...
                    0 0 0 1];
                HomoMat = HomoMat*A;
            end
            
            Eul = rotm2eul(HomoMat(1:3,1:3),'XYZ')./pi.*180;
            TCP = [HomoMat(1:3,4).' Eul];
        end
        
    end
    
end

