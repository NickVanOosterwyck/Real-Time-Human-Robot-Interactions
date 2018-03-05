%% select
ptCloudA = pointCloud(100*rand(1000,3,'single'));
roi = [0,50;0,inf;0,inf];
indices = findPointsInROI(ptCloudA, roi);
ptCloudB = select(ptCloudA,indices);

pcshow(ptCloudA.Location,'r');
hold on;
pcshow(ptCloudB.Location,'g');
hold off;

%% delete box
tic
ptCloud = cam.ptCloud;
XYZ = ptCloud.Location;
off = 0.1;
box = [-0.08-off 1.42+off -0.7-off 0.7+off 0-off 2.32+off];
k=1;
clear indices
for i = 1:length(XYZ)
    if XYZ(i,1)>box(1) &&  XYZ(i,1)<box(2) &&  XYZ(i,2)>box(3) &&  XYZ(i,2)<box(4) &&  XYZ(i,3)>box(5) &&  XYZ(i,3)<box(6)
        indices(k) = i;
        k = k+1;
    end
end

newptCloud = select(ptCloud,indices);
pcshow(newptCloud)
toc