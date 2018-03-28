cam = Kinect;
cam.Open('body');

[bodies, fcp, timeStamp] = cam.CameraProtocol.CameraSettings.getBodies('Euler');

%%
vid2 = videoinput('kinect',2,'Depth_512x424');

src = getselectedsource(vid2);

start(vid2);
[frame, ts, metaData] = getdata(vid2);

imagesc(metaData.SegmentationData);

%%
% Create color and depth kinect videoinput objects.
colorVid = videoinput('kinect', 1)
depthVid = videoinput('kinect', 2)

% Look at the device-specific properties on the depth source device,
% which is the depth sensor on the Kinect V2.
% Set 'EnableBodyTracking' to on, so that the depth sensor will
% return body tracking metadata along with the depth frame.
depthSource = getselectedsource(depthVid);
depthSource.EnableBodyTracking = 'on';

% Acquire 100 color and depth frames.
framesPerTrig = 100;
colorVid.FramesPerTrigger = framesPerTrig;
depthVid.FramesPerTrigger = framesPerTrig;

% Start the depth and color acquisition objects.
% This begins acquisition, but does not start logging of acquired data.
pause(5);
start([depthVid colorVid]);
