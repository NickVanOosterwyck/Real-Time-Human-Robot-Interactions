%% Add path
addpath(genpath(pwd)); % make sure current directory is the top map!

%% Create & Connect
RobotType = 'vrep';     % vrep or real

rob=ur10core(RobotType);

%% InvKin
JointPositions = [0 -80 -40 -180 -90 0];
[TCP,~,~] = rob.ForwKin(JointPositions);
%TCP = [664 -164 624 0 90 90];                     %output [0 -90 -90 -180 -90 0]
%TCP = [615.4006 -559.9200 108.8560 90 0 0];       %output [-25 -125 -100 -135 -25 0]

ang = [zeros(1,6) 0];
d = rob.DH.JointOffset;
a = rob.DH.LinkLength;
alf = rob.DH.LinkTwist;

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
H74 = inv(H47);
H01 = [cosd(ang(1)) -sind(ang(1))*cosd(alf(1)) sind(ang(1))*sind(alf(1)) a(1)*cosd(ang(1)); ...
        sind(ang(1)) cosd(ang(1))*cosd(alf(1)) -cosd(ang(1))*sind(alf(1)) a(1)*sind(ang(1)); ...
        0 sind(alf(1)) cosd(alf(1)) d(1); ...
        0 0 0 1];
p01 = H01(1:3,4);
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

H10 = inv(H01);
H14 = H01\H04; % H14 = H10*H04
ang(4) = -atan2d(-H14(2,3),H14(1,3))-270-ang(2)-ang(3);

ang(1:6)