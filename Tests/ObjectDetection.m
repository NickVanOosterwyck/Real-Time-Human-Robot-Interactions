%% init
cam=kinectcore('real');
cam.connect();

%% test
pause(5)
[ptCloud] = cam.getPointCloud('Filtered');
[ptCloudRaw] = cam.getPointCloud('Raw');
[bodies] = cam.getSkeleton();

%% plot
[f,ax]=cam.createAxis;
cam.drawSkeleton(bodies);
fig = gcf;
fig.PaperPositionMode = 'auto';
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_pos(3) fig_pos(4)];

[f2,ax2]=cam.createAxis;
pcshow(ptCloud,'MarkerSize',8);
%ax2.XLim=ax.XLim; ax2.YLim=ax.YLim;
fig = gcf;
fig.PaperPositionMode = 'auto';
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_pos(3) fig_pos(4)];

[f3,ax3]=cam.createAxis;
pcshow(ptCloudRaw,'MarkerSize',8);
%ax3.XLim=ax.XLim; ax3.YLim=ax.YLim;
fig = gcf;
fig.PaperPositionMode = 'auto';
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_pos(3) fig_pos(4)];

%% change camera
ax.CameraPosition = ax3.CameraPosition;
ax2.CameraPosition = ax3.CameraPosition;
ax.XLim = ax3.XLim; ax.YLim = ax3.YLim; ax.ZLim = ax3.ZLim;
ax2.XLim = ax3.XLim; ax2.YLim = ax3.YLim; ax2.ZLim = ax3.ZLim;



