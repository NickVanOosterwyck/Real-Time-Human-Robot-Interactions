% BODYDEMO Illustrates how to use the Kinect object to get and draw the
% detected people.
clear all
close all

%% setup camera

cam = Kinect;
cam.Open('body'); % tag to also calculate the body-pose
disp('Press q on any figure to exit')
k = 'n'; %% variable used to close the figure with a keypress

 [depth,color] = cam.GetFrame(TofFrameType.DEPTH_IMAGE,TofFrameType.RGB_IMAGE); % just get the first frames
        
%% setup figures

% depth stream figure
d.h = figure;
d.ax = axes;
d.im = imagesc(depth);
%hold on;

title('Depth Source (press q to exit)')
set(gcf,'keypress','k=get(gcf,''currentchar'');'); % listen keypress

% color stream figure
c.h = figure;
c.ax = axes;
c.im = imagesc(color);
title('Color Source (press q to exit)');
set(gcf,'keypress','k=get(gcf,''currentchar'');'); % listen keypress
%hold on

%% show

while true % note: never use a while loop without a brake-statement!
   
         [depth,color] = cam.GetFrame(TofFrameType.DEPTH_IMAGE,TofFrameType.RGB_IMAGE);  


        % update depth figure
        
        d.im = imagesc(fliplr(depth), 'Parent', d.ax);  % bones are calculated in the original kinect frame, which is mirrored. No problem with the XYZ-data. bodies.Position

       

        % update color figure
        
        c.im = imagesc(fliplr(color), 'Parent', c.ax);

        %set(c.im,'CData',color); 

        % Get 3D bodies joints 
        % Input parameter can be 'Quat' or 'Euler' for the joints
        % orientations.
        % getBodies returns a structure array.
        % The structure array (bodies) contains 6 bodies at most
        % Each body has:
        % -Position: 3x25 matrix containing the x,y,z of the 25 joints in
        %   camera space coordinates
        % - Orientation: 
        %   If input parameter is 'Quat': 4x25 matrix containing the 
        %   orientation of each joint in [x; y; z, w]
        %   If input parameter is 'Euler': 3x25 matrix containing the 
        %   orientation of each joint in [Pitch; Yaw; Roll] 
        % -TrackingState: state of each joint. These can be:
        %   NotTracked=0, Inferred=1, or Tracked=2
        % -LeftHandState: state of the left hand
        % -RightHandState: state of the right hand
        [bodies, fcp, timeStamp] = cam.CameraProtocol.CameraSettings.getBodies('Quat');   %% note: this will calculate the bodies in the images that are in the buffer: the last time you called cam.GetFrame(....)     
        
        % Number of bodies detected
        numBodies = size(bodies,2);
        %disp(['Bodies Detected: ' num2str(numBodies)])
       
        % Example of how to extract information from getBodies output.
       if numBodies > 0
          
            
            disp('Floor Clip Plane')
            disp(fcp);
            
            disp('Body Timestamp')
            disp(timeStamp);
        
            % To get the joints on depth image space, you can use:
            %pos2D =
            %cam.CameraProtocol.CameraSettings.mapCameraPoints2Depth(bodies(1).Position');
            
            
        end
         
        %To get the joints on color image space, you can use:
        %pos2D = cam.CameraProtocol.CameraSettings.mapCameraPoints2Color(bodies(1).Position');

        % Draw bodies on depth image
        % Parameters: 
        % 1) image axes
        % 2) bodies structure
        % 3) Destination image (depth or color)
        % 4) Joints' size (circle raddii)
        % 5) Bones' Thickness
        % 6) Hands' Size
        cam.CameraProtocol.CameraSettings.drawBodies(d.ax,bodies,'depth',5,3,15);
        
        % Draw bodies on color image
        cam.CameraProtocol.CameraSettings.drawBodies(c.ax,bodies,'color',10,6,30);
        
    
    
    % If user presses 'q', exit loop
    if ~isempty(k)
        if strcmp(k,'q'); break; end;
    end
  
    pause(0.02)
end

