%% Set up
cam=kinectcore('real');    % choose kinect
cam.connect();
rob=ur10core('vrep');    %-- choose UR10;
rob.connect();

cam.moveToCameraLocation([-0.5 0 0.83 0 90 90]);
cam.detectionVol = [0 1.4 -1.5 1 0 2];

[ptCloudFiltered] = cam.getFilteredPointCloud();
cam.showPointCloud(ptCloudFiltered);
%% Calculate average distance
pause(2)
% init
n = 10; clc
dist = zeros(1,n);
t = zeros(1,n);

% get frames
for i = 1:n
    tic
    [dist(i),~] = cam.getClosestPoint();
    t(i) = toc;
    disp(['Frame (', num2str(i), '/', num2str(n),')'])
end
av_dist = mean(dist);
av_t = mean(t);
disp(['Average distance is ',num2str(av_dist) ])
disp(['Average time is ',num2str(av_t) ])

% plot
figure
hold on
plot(dist)
plot(t)
if sum(dist==Inf)==n
    text(n/2,1.5,'No objects detected')
    
else
    text(n/2,av_dist+0.1,[num2str(av_dist) ' m'])
end
text(n/2,av_t+0.1,[num2str(av_t) ' s'])
hold off
axis([1 n 0 3])
legend('distance','time')





