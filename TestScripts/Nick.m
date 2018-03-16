%% Init
cam = 'vrep';
%cam = 'real';

rob = 'vrep';
%rob = 'real';

ctrl = controller(cam,rob);
ctrl.connect();

%% Test
ctrl.cam.showPlayer();
