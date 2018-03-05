function [] = Show3D(ptCloud, Limits)
% Show3D creates 3D image from a pointcloud en displays a line from the
% camera to the closest point
%--------------------------------------------------------------------------
% INPUTS
% ptCloud     Pointcloud with color [1x1 pointCloud]
% Limits      Limits of 3D image [1x6 double]
%--------------------------------------------------------------------------

%-- determine closest point
    [indices,~] = findNearestNeighbors(ptCloud,[0 0 0],1);
    ClosestPoint = [ptCloud.Location(indices,1) ptCloud.Location(indices,2) ptCloud.Location(indices,3)];
    
%-- show pointcloud
    figure
    pcshow(ptCloud)
    axis equal
    axis([Limits])
    camorbit(180,0,'data',[0 1 0])
    camorbit(-90,0,'data',[0 0 1])
    camorbit(90,0,'data',[1 0 0])
    camorbit(-30,0,'data',[0 1 0])
    title('3D')
    xlabel('X [m]');
    ylabel('Y [m]');
    zlabel('Z [m]');
    
%-- draw line to closest point
    hold on
    plot3([0 ClosestPoint(1)],[0 ClosestPoint(2)],[0 ClosestPoint(3)],'r')
end

