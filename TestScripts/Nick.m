%% Create & connect
CameraType = 'real';    % vrep or real
RobotType = 'real';     % vrep or real

ctrl = controller(CameraType,RobotType);
ctrl.connect();

%%
pause(5)
bodies=ctrl.cam.getSkeleton();
h=ctrl.cam.getHandHeight('Left','Max',bodies)

ctrl.cam.createAxis();
hold on
n=ctrl.cam.drawSkeleton(bodies);
hold off

%%
pause(5)
ctrl.showDistanceCalculation('Skeleton','TCP'); % ptCloud/Skeleton and TCP/Base

%%
pause(5)
h=ctrl.cam.getHandHeight('Right','Max')

%%
ctrl.showTrackingPlayer('Skeleton','TCP')

%%
ctrl.rob.goHome()
ctrl.rob.setSpeedFactor(0);
