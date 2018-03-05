function [] = ShowPlayer(cam, axislimits)
% ShowPlayer creates a 3D live player from pointclouds. note: player is
% automatically terminated when window is closed
%--------------------------------------------------------------------------
% INPUTS
% cam         Camera object [VREP_Projetor or Kinect]
% axisLimits  Axis limits of 3D image [1x6 double] note: displayed limits
%             can be different than size of ptCloud
%--------------------------------------------------------------------------
%-- load the camera viewpoint
    load('CameraPosition');
    
%-- create player and set the camera viewpoint
    player = pcplayer(axislimits(1:2),axislimits(3:4),axislimits(5:6));
    player.Axes.CameraPositionMode = 'manual';
    player.Axes.CameraPosition = AxesCamera.CameraPosition;
    player.Axes.CameraTargetMode = 'manual';
    player.Axes.CameraTarget = AxesCamera.CameraTarget;
    player.Axes.CameraUpVectorMode = 'manual';
    player.Axes.CameraUpVector = AxesCamera.CameraUpVector;

%-- label axes
    xlabel(player.Axes,'X [m]');
    ylabel(player.Axes,'Y [m]');
    zlabel(player.Axes,'Z [m]');
   
% -- V-REP: update while the player is open 
    if strcmp(class(cam),'VREP_Projector')
     while isOpen(player)
          [XYZ_ORG,RGB] = cam.GetFrame(TofFrameType.XYZ_ORGANIZED,TofFrameType.RGB_IMAGE);
          ptCloud = createPointCloud(XYZ_ORG,RGB,axislimits);
          view(player, ptCloud);
     end 
    end
% -- Kinect: update while the player is open
    if strcmp(class(cam),'Kinect')
     while isOpen(player)
          ptCloud = cam.GetFrame(TofFrameType.POINTCLOUD_COLOR);
          view(player, ptCloud);
     end 
    end
%-- close figures
    close all
end
