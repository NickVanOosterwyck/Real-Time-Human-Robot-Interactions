function [] = ShowImages(RGB, DEPTH, INFRARED, ptCloud, Limits)
% ShowImages creates a matlab-figure containing an RGB, DEPTH, INFRARED and
% 3D image
%--------------------------------------------------------------------------
% INPUTS
% RGB         RGB-image in camera toolbox format [nxmx3 uint8]
% DEPTH       Depth-image in camera toolbox format [nxm single]
% INFRARED    Infrared-image in camera toolbox format [nxm uint8]
% ptCloud     Pointcloud with color [1x1 pointCloud]
% Limits      Limits of 3D image [1x6 double]
%--------------------------------------------------------------------------

figure('Name','Kinect Images')

% RGB
subplot(2,2,1);
imagesc(RGB);
axis image
title('RGB image')

% Depth
subplot(2,2,2);
imagesc(DEPTH);
axis image
title('Depth image')

% Infrared
subplot(2,2,3);
imagesc(INFRARED);
axis image
title('Infrared image')

% 3D
subplot(2,2,4);
pcshow(ptCloud);
axis equal
axis(Limits)
camorbit(180,0,'data',[0 1 0])
camorbit(-90,0,'data',[0 0 1])
camorbit(90,0,'data',[1 0 0])
camorbit(-30,0,'data',[0 1 0])
title('3D')
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');

end

