%% Livestream RGB
f2=figure(2);
while ishandle(f2)
    % RGB
    subplot(2,1,1)
    imagesc(cam.GetFrame(TofFrameType.RGB_IMAGE));
    axis image
    title('RGB image LIVE')
    drawnow limitrate
end
close all
%% test
% Simple BodyDEMO Illustrates how to use the Kinect object to get and draw the
% detected people.

load('camera_parameters');
limits = [-2 2 -0.8 1.4 0 3.5];

cam = VREP_Projector(); % This is just a camera. You can not 'project' images with this. It does however use cx and cy of the intrinsic parameters. The virtual sensor in vrep will look wrong, but in Matlab you will get the correct image.
cam.Open('Kinect_sensor')
cam.setParams(copy(Ip)); % set the intrinsics of the projector


%{
cam = Kinect;
cam.Open('body');
%}


for k = 1: 100 % 100 frames record
    
    [depth,color] = cam.GetFrame(TofFrameType.DEPTH_IMAGE,TofFrameType.RGB_IMAGE);
    [bodies, fcp, timeStamp] = cam.CameraProtocol.CameraSettings.getBodies('Quat'); %'Quat': 4x25 matrix containing the  orientation of each joint in [x; y; z, w] (quaternions representation of angles)
    
    
    numBodies = size(bodies,2);
    
    
    figure(1)
    hold off
    imagesc(fliplr(depth)) % bones are calculated in the original kinect frame, which is mirrored. No problem with the XYZ-data. bodies.Position
        
        
        
    if numBodies > 0
        
        
        disp('Floor Clip Plane')
        disp(fcp);
        
        disp('Body Timestamp')
        disp(timeStamp);
        
        % To get the joints on depth image space, you can use:
        %pos2D =
        DepthPositions = (cam.CameraProtocol.CameraSettings.mapCameraPoints2Depth(bodies(1).Position'));
        
        
        
        [bonesx, bonesy] = cam.CameraProtocol.CameraSettings.getBones(DepthPositions);
        
        
        i = 1; % only the first body
        
        
        
        % Draw the bones
       
        hold on
        for j=1:24
            line(bonesx(:,j),bonesy(:,j),'Color',cam.CameraProtocol.CameraSettings.bodyColors(i), ...
                'LineWidth',2);
        end
        
        
    end
    drawnow;
        pause(0.3)
end