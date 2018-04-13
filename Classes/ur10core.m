classdef ur10core < handle
    %ur10core Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected)
        rob                         % Selected ur10 class
        DH                          % Denavit-Hartenberg parameters
        TCPoffset                   % Distance between TCP and robot end
        homeJointTargetPositions;   % Joint positions of home pose in degree
        JointTargetPositions        % Joint target postions in degree
        TCPTargetPositions          % Position of TCP [x y z alfa beta gamma] (eul: ZYX absolute axes/ XYZ own axes)
        MaxSpeedFactor              % Limiting factor of the joint speeds
    end
    
    methods
        function obj = ur10core(RobotType)
            p = inputParser;
            acceptedInput = {'vrep','real'};
            p.addRequired('RobotType',@(x) any(validatestring(x,acceptedInput)));
            p.parse(RobotType);
            if p.Results.RobotType == 'vrep'
                obj.rob = ur10vrep();
            else
                obj.rob = ur10real();
            end
            obj.homeJointTargetPositions = [0 -90 -90 -180 -90 0];
            obj.JointTargetPositions = zeros(1,6);
            obj.TCPTargetPositions = zeros(1,6);
            obj.MaxSpeedFactor = 1;
            obj.TCPoffset = 0;
            obj.DH.JointOffset = [128 0 0 164 116 92 obj.TCPoffset];
            obj.DH.LinkLength = [0 -612 -572 0 0 0 0];
            obj.DH.LinkTwist = [90 0 0 90 -90 0 0];
        end % constructor
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
        function set.MaxSpeedFactor(obj,Speed)
            if isnumeric(Speed) && Speed<=1 && Speed>=0
                obj.MaxSpeedFactor = Speed;
            else
                error('Invalid MaxSpeedFactor!\n%s',...
                    'Use values between 0 and 1.')
            end
        end
        function [TCPTargetPositions] = get.TCPTargetPositions(obj)
            [TCPTargetPositions,~,~]=obj.ForwKin(obj.JointTargetPositions); 
        end
        
        function connect(obj)
            obj.rob.connect();
            [~] = obj.getJointPositions();
            pause(0.1)
            startPositions = obj.getJointPositions(); %2nd call because of streaming operation mode in VREP
            obj.JointTargetPositions=startPositions;
            [obj.TCPTargetPositions,~,~]=obj.ForwKin(startPositions);
        end
        function [JointPositions] = getJointPositions(obj)
            JointPositions_rad=obj.rob.get_actual_joint_positions();
            JointPositions=JointPositions_rad/pi*180;
        end
        function [TCPSpeed] = getTCPSpeed(obj)
            Velocities = obj.rob.get_actual_tcp_speed();
            TCPSpeed = sqrt(Velocities(1)^2+Velocities(2)^2+Velocities(3)^2);
        end
        function movej(obj,q,a,v,t,r,varargin)
            p=inputParser;
            acceptedInput = {'Joint','World'};
            p.addRequired('obj');
            p.addRequired('q');
            p.addRequired('a');
            p.addRequired('v');
            p.addRequired('t');
            p.addRequired('r');
            p.addOptional('CS','Joint',@(x) any(validatestring(x,acceptedInput))); %coordinate system
            p.parse(obj,q,a,v,t,r,varargin{:});
            
            if p.Results.CS =='World'
                q=obj.InvKin(q);
            end
            obj.JointTargetPositions=q;
            q=q/180*pi;
            obj.rob.movej(q,a,v,t,r)
        end
        function movel(obj,q,a,v,t,r,varargin)
            p=inputParser;
            acceptedInput = {'Joint','World'};
            p.addRequired('obj');
            p.addRequired('q');
            p.addRequired('a');
            p.addRequired('v');
            p.addRequired('t');
            p.addRequired('r');
            p.addOptional('CS','Joint',@(x) any(validatestring(x,acceptedInput))); %coordinate system
            p.parse(obj,q,a,v,t,r,varargin{:});
            
            if p.Results.CS =='World'
                q=obj.InvKin(q);
            end
            q=q/180*pi;
            if isa(obj.rob,'ur10vrep')
                obj.rob.movej(q,a,v,t,r) %movel is not supported in VREP
            else
                obj.rob.movel(q,a,v,t,r)
            end
        end
        function stopj(obj,a)
            obj.rob.stopj(a);
        end
        function goHome(obj,varargin)
            p=inputParser;
            p.addRequired('obj');
            p.addOptional('Wait',false,@islogical);
            p.parse(obj,varargin{:});
            
            obj.setSpeedFactor(1)
            obj.movej(obj.homeJointTargetPositions,0.5,0.1,0,0);
            if p.Results.Wait
                while ~obj.checkPoseReached(obj.homeJointTargetPositions,0.1)
                end
            end
        end
            

        function setSpeedFactor(obj,SpeedFactor)
            obj.MaxSpeedFactor = SpeedFactor;
            obj.rob.setSpeedFactor(SpeedFactor);
            pause(0.1)
        end
        function [flag] = checkPoseReached(obj,JointTargetPositions,range)
            Positions = obj.getJointPositions();
            if max(abs(Positions-JointTargetPositions))< range
                flag=1;
            else
                flag=0;
            end
        end
        function [TCP,R,T] = ForwKin(obj,JointPositions)
            ang = [JointPositions 0];
            d = obj.DH.JointOffset;
            a = obj.DH.LinkLength;
            alf = obj.DH.LinkTwist;
            
            % transformation i-1 to i
            R(:,:,1) = eye(4,4);
            for i=1:7
                R(:,:,i) = [cosd(ang(i)) -sind(ang(i))*cosd(alf(i)) sind(ang(i))*sind(alf(i)) a(i)*cosd(ang(i)); ...
                    sind(ang(i)) cosd(ang(i))*cosd(alf(i)) -cosd(ang(i))*sind(alf(i)) a(i)*sind(ang(i)); ...
                    0 sind(alf(i)) cosd(alf(i)) d(i); ...
                    0 0 0 1];
            end
            
            % transformation 0 to i
            T(:,:,1) = R(:,:,1);
            for i=2:7
                T(:,:,i) = T(:,:,i-1)*R(:,:,i);
            end
            
            Eul = rotm2eul(T(1:3,1:3,7),'XYZ')./pi.*180;
            TCP = [T(1:3,4,7).' Eul];
        end
        function JointPositions = InvKin (obj,TCP)
            ang = [zeros(1,6) 0];
            d = obj.DH.JointOffset;
            a = obj.DH.LinkLength;
            alf = obj.DH.LinkTwist;
            
            p07 = TCP(1:3)';
            R07 = eul2rotm(TCP(4:6).*pi./180,'XYZ');
            R07 = round(R07,4);
            H07 = [R07 p07;zeros(1,3) 1];
            
            p05 = p07-(d(6).*R07(:,3));
            
            ang(1) = atan2d(p05(2),p05(1))-acosd(d(4)/sqrt(p05(2)^2+p05(1)^2))+90;
            ang(5) = -acosd(((TCP(1)*sind(ang(1)))-(TCP(2)*cosd(ang(1)))-d(4))/d(6));
            ang(6) = atan2d((-R07(1,2)*sind(ang(1))+(R07(2,2)*cosd(ang(1))))/sind(ang(5)),-(-R07(1,1)*sind(ang(1))+(R07(2,1)*cosd(ang(1))))/sind(ang(5)));
            
            H47 = eye(4,4);
            for i=5:7
                R = [cosd(ang(i)) -sind(ang(i))*cosd(alf(i)) sind(ang(i))*sind(alf(i)) a(i)*cosd(ang(i)); ...
                    sind(ang(i)) cosd(ang(i))*cosd(alf(i)) -cosd(ang(i))*sind(alf(i)) a(i)*sind(ang(i)); ...
                    0 sind(alf(i)) cosd(alf(i)) d(i); ...
                    0 0 0 1];
                H47 = H47*R;
            end
            H01 = [cosd(ang(1)) -sind(ang(1))*cosd(alf(1)) sind(ang(1))*sind(alf(1)) a(1)*cosd(ang(1)); ...
                sind(ang(1)) cosd(ang(1))*cosd(alf(1)) -cosd(ang(1))*sind(alf(1)) a(1)*sind(ang(1)); ...
                0 sind(alf(1)) cosd(alf(1)) d(1); ...
                0 0 0 1];
            p01 = H01(1:3,4);
            %H74 = inv(H47);
            H04 = H07/H47; %H04 = H07*H74
            p04 = H04(1:3,4);
            p03 = p04-(d(4).*H04(1:3,2));
            
            %2D planar between p01 and p03
            xr = sqrt(p03(1)^2+p03(2)^2); % horizontal distance between
            yr = p03(3)-p01(3);
            l1 = -a(2);
            l2 = -a(3);
            ang(3) = atan2d(-sqrt(1-((xr^2+yr^2-l1^2-l2^2)/(2*l1*l2))^2),(xr^2+yr^2-l1^2-l2^2)/(2*l1*l2));
            ang(2) = atan2d(sqrt(1-(((xr*(l1+(l2*cosd(ang(3)))))+(yr*l2*sind(ang(3))))/(xr^2+yr^2))^2),((xr*(l1+(l2*cosd(ang(3)))))+(yr*l2*sind(ang(3))))/(xr^2+yr^2));
            ang(2) = ang(2)-180;
            
            %H10 = inv(H01);
            H14 = H01\H04; % H14 = H10*H04
            ang(4) = -atan2d(-H14(2,3),H14(1,3))-270-ang(2)-ang(3);
            
            JointPositions = ang(1:6);
        end
        function drawRobot(obj)
            [~,~,T] = obj.ForwKin(obj.getJointPositions);
            
            P = zeros(7,3);
            for i=1:7
                P(i,:) = [T(1,4,i)/1000 T(2,4,i)/1000 T(3,4,i)/1000+0.86];
                %plot3(P(i,1),P(i,2),P(i,3),'g','Marker','o','LineWidth',2)
            end
            
            plot3([0 P(1,1)],[0 P(1,2)],[0.86 P(1,3)],'g','Marker','o','LineWidth',2)
            for i=2:7
                plot3([P(i-1,1) P(i,1)],[P(i-1,2) P(i,2)],[P(i-1,3) P(i,3)],'g','Marker','o','LineWidth',2)
            end
            
        end
        
    end
    
end

