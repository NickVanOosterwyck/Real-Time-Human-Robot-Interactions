Rotm = [0 0 1  ;...
        0 1 0  ; ...
        -1 0 0 ]; 
 Eul = rotm2eul(Rotm,'XYZ')./pi*180
 Rotm2 = eul2rotm(Eul./180.*pi,'XYZ')
 
 camera = plotCamera('Location',[0, 0, 0],'Orientation',Rotm.','Opacity',0,'AxesVisible',true);
 hold on
 quiver3(0,0,0,1,0,0,0.3,'r','Linewidth',1.5)
 quiver3(0,0,0,0,1,0,0.3,'g','Linewidth',1.5)
 quiver3(0,0,0,0,0,1,0.3,'b','Linewidth',1.5)
 xlabel('X [m]');
 ylabel('Y [m]');
 zlabel('Z [m]');
 hold off