%% connect
rosshutdown
rosinit('http://192.168.1.100:11311', 'NodeHost', '192.168.1.16')
jointsub= rossubscriber('/joint_states');
scriptpub= rospublisher('/ur_driver/URScript');

%% get position
JointPositions_rad = jointsub.LatestMessage.Position.';
JointPositions=JointPositions_rad/pi*180;

%% Digital Out ?
msg=rosmessage('std_msgs/String');
msg.Data = 'set_digital_out(2,False)';
send(scriptpub,msg);

%% Move
%home
msg=rosmessage('std_msgs/String');
msg.Data = 'movej([0,-1.5708,-1.5708,-3.1416,-1.5708,0],1,0.1,0,0)';
send(scriptpub,msg);

%% Move2
q = [0.7854 -1.9774 -0.7814 -3.5244 -2.3562 0];
a=1; v=0.1; t=0; r=0;
msg=rosmessage('std_msgs/String');
q_str = ['[' num2str(q(1)) ',' num2str(q(2)) ',' num2str(q(3)) ',' num2str(q(4)) ',' num2str(q(5)) ',' num2str(q(6)) ']'];
msg.Data = ['movej(' q_str ',' num2str(a) ',' num2str(v) ',' num2str(t) ',' num2str(r) ')'];
send(scriptpub,msg);

%% Speed 
SpeedFactor =0.90;
msg=rosmessage('std_msgs/String');
msg.Data = ['set speed' num2str(SpeedFactor)];
send(scriptpub,msg);



