%%
pause(5)
bodies=cam.getSkeleton();
h=cam.getHandHeight('Left',bodies)
[Dist,Start,End] = ctrl.getClosestPoint('Skeleton','TCP',bodies);

cam.createAxis();
hold on
n=cam.drawSkeleton(bodies);
plot3(Start(1),Start(2),Start(3),'o');
plot3(End(1),End(2),End(3),'o');
hold off

%%
pause(5)
ctrl.showDistanceCalculation('Skeleton','Base'); % ptCloud/Skeleton and TCP/Base