%% init
ctrl = controller('real','real');
ctrl.connect();

%% plot
pause(5)
[f1,ax1]=ctrl.showDistanceCalculation('Skeleton','TCP');
fig = gcf;
fig.PaperPositionMode = 'auto';
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_pos(3) fig_pos(4)];
[f2,ax2]=ctrl.showDistanceCalculation('ptCloud','TCP');
fig = gcf;
fig.PaperPositionMode = 'auto';
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_pos(3) fig_pos(4)];

%% edit
ax2.XLim=ax1.XLim; ax2.YLim=ax1.YLim; ax2.ZLim=ax1.ZLim;
ax2.CameraPosition=ax1.CameraPosition;