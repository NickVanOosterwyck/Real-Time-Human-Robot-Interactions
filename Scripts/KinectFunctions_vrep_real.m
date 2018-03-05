%% Clear
clear; close all; clc

%% Add path
% Add folder and subfolders to the search path
addpath(genpath('C:\Users\Nick\Desktop\Master Project'));

%% Open camera (V-REP)
cam = VREP_Projector(); % This is just a camera. You can not 'project' images with this. It does however use cx and cy of the intrinsic parameters. The virtual sensor in vrep will look wrong, but in Matlab you will get the correct image.
cam.Open('Kinect_sensor');
load('camera_parameters');
cam.setParams(copy(Ip)); % set the intrinsics of the projector

%% Open camera (Real)
cam = Kinect();
cam.Open()

%% Get camera pose
Hp = cam.getPose(); % get the pose of a camera/projector.

%% Show images
% note: Simulation in V-REP must be running when executing the folowwing commands!!!
%-- get images
[XYZ_ORG,RGB,DEPTH,INFRARED] = cam.GetFrame(TofFrameType.XYZ_ORGANIZED,TofFrameType.RGB_IMAGE,TofFrameType.DEPTH_IMAGE,TofFrameType.INFRARED_IMAGE);

%-- set limits for filtering and create filtered pointcloud
Limits = [-2 2 -0.8 1.4 0 3.5];
ptCloud = createPointCloud(XYZ_ORG,RGB,Limits);

%-- show images
ShowImages(RGB,DEPTH,INFRARED,ptCloud,Limits);
Show3D(ptCloud,Limits);

%% Show 3D with player
% note: Simulation in V-REP must be running when executing the folowwing commands!!!
Limits = [-2 2 -0.8 1.4 0 3.5];
ShowPlayer(cam,Limits);
