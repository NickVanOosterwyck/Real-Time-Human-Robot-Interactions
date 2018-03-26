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
msg.Data = 'set_digital_out(2,True)\n';
send(scriptpub,msg);

%% Move ?
%home
msg=rosmessage('std_msgs/String');
msg.Data = 'movej([0,-1.5708,-1.5708,-3.1416,-1.5708,0],1,0.1,0,0)\n';
send(scriptpub,msg);

%% Move2 ?
q = [0,-1.5708,-1.5708,-3.1416,-1.5708,0];
a=1; v=0.1; t=0; r=0;
msg=rosmessage('std_msgs/String');
q_str = ['[' num2str(q(1)) ',' num2str(q(2)) ',' num2str(q(3)) ',' num2str(q(4)) ',' num2str(q(5)) ',' num2str(q(6)) ']'];
msg.Data = ['movej(' q_str ',' num2str(a) ',' num2str(v) ',' num2str(t) ',' num2str(r) ')\n'];

%% Speed ?
SpeedFactor =0.5;
msg1=rosmessage('std_msgs/String');
msg2=rosmessage('std_msgs/String');
msg3=rosmessage('std_msgs/String');
msg1.Data = 'set speed\n';
msg2.Data = num2str(SpeedFactor);
msg3.Data = '10\n';
send(scriptpub,msg1);
send(scriptpub,msg2);
send(scriptpub,msg3);
