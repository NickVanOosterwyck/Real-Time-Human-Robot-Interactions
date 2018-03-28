% Simple BodyDEMO Illustrates how to use the Kinect object to get and draw the
% detected people.



cam = Kinect;
cam.Open('body');
%%

for k = 1: 100 % 100 frames record
    
    [depth,color] = cam.GetFrame(TofFrameType.DEPTH_IMAGE,TofFrameType.RGB_IMAGE);
    [bodies, fcp, timeStamp] = cam.CameraProtocol.CameraSettings.getBodies('Euler'); %'Quat': 4x25 matrix containing the  orientation of each joint in [x; y; z, w] (quaternions representation of angles)
    
    
    numBodies = size(bodies,2);
    
    
    figure(1)
    hold off
    imagesc(fliplr(color)) % bones are calculated in the original kinect frame, which is mirrored. No problem with the XYZ-data. bodies.Position
        
        
        
    if numBodies > 0
        
        
        disp('Floor Clip Plane')
        disp(fcp);
        
        disp('Body Timestamp')
        disp(timeStamp);
        
        % To get the joints on depth image space, you can use:
        %pos2D =
        DepthPositions = (cam.CameraProtocol.CameraSettings.mapCameraPoints2Color(bodies(1).Position'));
        
        
        
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
        %pause(0.3)
end