%%
CameraType = 'real';    % vrep or real

cam=kinectcore(CameraType);
cam.connect();
%%
pause(5)
bodies=cam.getSkeleton();
%h=cam.getHandHeight('Left',bodies)

cam.createAxis();
hold on
n=cam.drawSkeleton(bodies);
hold off

%%
pause(5)
ctrl.showDistanceCalculation('Skeleton','Base'); % ptCloud/Skeleton and TCP/Base

%%
pause(5)
[depth,color] = cam.GetFrame(TofFrameType.DEPTH_IMAGE,TofFrameType.RGB_IMAGE);
[bodies, fcp, timeStamp] = cam.CameraProtocol.CameraSettings.getBodies('Euler');
