%% Add path
addpath(genpath(pwd)); % make sure current directory is the top map!

%% Clear
clear; close all; clc

%% Connect
cam=kinectreal();
cam.connect();

%% Get RGB images
cd Data\CalibrationImages
for i= 1:20
    disp(['Image ',num2str(i)]);
    pause(2)
    RGB = cam.getRGB();
    imwrite(RGB,['Image',num2str(i),'.png'])
end
cd ../../

%% Launch Calibrator
cameraCalibrator
