%%
pause(5)
bodies=cam.getSkeleton();
h=cam.getHandHeight('Left',bodies)

cam.createAxis();
hold on
n=cam.drawSkeleton(bodies);
hold off