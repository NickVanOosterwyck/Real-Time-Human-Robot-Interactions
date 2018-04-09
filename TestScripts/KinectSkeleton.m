%% Setup the Kinect V2 for color and depth acquisition.
% Create color and depth kinect videoinput objects.
%colorVid = videoinput('kinect', 1);
depthVid = videoinput('kinect', 2);

% Look at the device-specific properties on the depth source device,
% which is the depth sensor on the Kinect V2.
% Set 'EnableBodyTracking' to on, so that the depth sensor will
% return body tracking metadata along with the depth frame.
depthSource = getselectedsource(depthVid);
depthSource.EnableBodyTracking = 'on';

% Acquire 100 color and depth frames.
framesPerTrig = 1;
%colorVid.FramesPerTrigger = framesPerTrig;
depthVid.FramesPerTrigger = framesPerTrig;
triggerconfig(depthVid);
%triggerconfig([colorVid depthVid],'manual');

% Preview
preview(depthVid);
closepreview(depthVid);

% Start the depth and color acquisition objects.
% This begins acquisition, but does not start logging of acquired data.
start(depthVid);

%%
% Get images and metadata from the color and depth device objects.
%[colorImg] = getdata(colorVid);
%trigger(depthVid)
pause(5)
tic
[~, ~, metadata] = getdata(depthVid);
toc
start(depthVid);
toc
%%
% These are the order of joints returned by the kinect adaptor.
%    SpineBase = 1;
%    SpineMid = 2;
%    Neck = 3;
%    Head = 4;
%    ShoulderLeft = 5;
%    ElbowLeft = 6;
%    WristLeft = 7;
%    HandLeft = 8;
%    ShoulderRight = 9;
%    ElbowRight = 10;
%    WristRight = 11;
%    HandRight = 12;
%    HipLeft = 13;
%    KneeLeft = 14;
%    AnkleLeft = 15;
%    FootLeft = 16;
%    HipRight = 17;
%    KneeRight = 18;
%    AnkleRight = 19;
%    FootRight = 20;
%    SpineShoulder = 21;
%    HandTipLeft = 22;
%    ThumbLeft = 23;
%    HandTipRight = 24;
%    ThumbRight = 25;

%% Access Image and Skeletal Data.
% Create skeleton connection map to link the joints.
SkeletonConnectionMap = [ [4 3];  % Neck
                          [3 21]; % Head
                          [21 2]; % Right Leg
                          [2 1];
                          [21 9];
                          [9 10];  % Hip
                          [10 11];
                          [11 12]; % Left Leg
                          [12 24];
                          [12 25];
                          [21 5];  % Spine
                          [5 6];
                          [6 7];   % Left Hand
                          [7 8];
                          [8 22];
                          [8 23];
                          [1 17];
                          [17 18];
                          [18 19];  % Right Hand
                          [19 20];
                          [1 13];
                          [13 14];
                          [14 15];
                          [15 16];
                        ];

% Extract the 90th frame and tracked body information.
lastFrame = framesPerTrig;
lastframeMetadata = metadata(lastFrame);

% Find the indexes of the tracked bodies.
anyBodiesTracked = any(lastframeMetadata.IsBodyTracked ~= 0);
trackedBodies = find(lastframeMetadata.IsBodyTracked);

% Find number of Skeletons tracked.
nBodies = length(trackedBodies);

% Get the joint indices of the tracked bodies with respect to the color
% image.
colorJointIndices = lastframeMetadata.ColorJointIndices(:, :, trackedBodies);

% Extract the 90th color frame.
lastColorImage = colorImg(:, :, :, lastFrame);

%% View RGB image with skeletal overlay
% Marker colors for up to 6 bodies.
colors = ['r';'g';'b';'c';'y';'m'];

% Display the RGB image.
imshow(lastColorImage);

% Overlay the skeleton on this RGB frame.
for i = 1:24
     for body = 1:nBodies
         X1 = [colorJointIndices(SkeletonConnectionMap(i,1),1,body) colorJointIndices(SkeletonConnectionMap(i,2),1,body)];
         Y1 = [colorJointIndices(SkeletonConnectionMap(i,1),2,body) colorJointIndices(SkeletonConnectionMap(i,2),2,body)];
         line(X1,Y1, 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '+', 'Color', colors(body));
     end

    hold on;
 end
 hold off;
 
 %% Delete
 delete(colorVid);
 delete(depthVid);
 
 