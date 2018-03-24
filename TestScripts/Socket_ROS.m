%% connect
rosshutdown
rosinit('http://192.168.1.100:11311', 'NodeHost', '192.168.1.16')
jsub= rossubscriber('/joint_states');
scriptpub= rospublisher('/ur_driver/URScript');

%% get position
JointPositions_rad = jsub.LatestMessage.Position.';
JointPositions=JointPositions_rad/pi*180;

%% Digital Out
msg=rosmessage('std_msgs/String');
msg.Data = 'set_digital_out(2,True)\n';
send(scriptpub,msg);

%% Move
%home
msg=rosmessage('std_msgs/String');
msg.Data = 'movej([0,-1.5708,-1.5708,-3.1416,-1.5708,0],a=3,v=0.1,t=0,r=0)\n';
send(scriptpub,msg);
