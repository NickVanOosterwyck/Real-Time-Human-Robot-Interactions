%% Calculate average numbers
pause(2)
% init
n = 10; clc
observed = zeros(1,n);
detected = zeros(1,n);
rStop = 3/2;
threshold = 100;

% get frames
for i = 1:n
    tic
[ptCloudFiltered] = cam.getFilteredPointCloud();
observed(i) = ptCloudFiltered.Count;
[indices, dist] = findNeighborsInRadius(ptCloudFiltered,[0 0 1],2.5);
toc
detected(i) = sum(dist<rStop);
disp(['Frame (', num2str(i), '/', num2str(n),')'])
end
[ptCloudRaw] = cam.getRawPointCloud();
av = mean(observed);
disp(['Average is ',num2str(av) ])

% post processing


% plot
figure
plot(observed)
axis([1 n 0 max(observed)])    
hold on
plot([1 n],[av av])
hold off
%% Plot pointclouds
% plot latest frame
cam.showPointCloud(ptCloudRaw);
cam.showPointCloud(ptCloudFiltered)
