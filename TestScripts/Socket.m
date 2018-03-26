%% connect
echotcpip('on',30002)
t = tcpip('192.168.1.110', 30002, 'NetworkRole', 'client');
fopen(t);

%% Digital Out
fprintf(t,'set_digital_out(2,True)\n');
fprintf(t,'set_digital_out(2,False)\n');

%% Movej
%home
fprintf(t,'movej([0,-1.5708,-1.5708,-3.1416,-1.5708,0],a=0.5,v=0.1,t=0,r=0)\n');
%pick up
fprintf(t,'movej([0.7854,-1.9774,-0.7814,-3.5244,-2.3562,0],a=0.5,v=0.1,t=0,r=0)\n');
%place
fprintf(t,'movej([-0.4363,-1.9774,-0.7814,-3.5244,-0.4363,0],a=0.5,v=0.3,t=0,r=0)\n');
fprintf(t,'movej([-0.4363,-1.9774,-0.7814,-3.5244,-0.4363,0],a=0.5,v=0.1,t=0,r=0)\n');

%% Movel
%home
fprintf(t,'movel([0,-1.5708,-1.5708,-3.1416,-1.5708,0],a=0.5,v=0.1,t=0,r=0)\n');
fprintf(t,'movel([0,-1.5708,-1.5708,-3.1416,-1.5708,0],0.5,0.1,0,0)\n'); %?

%% Stop
fprintf(t,'stopj(0.5)\n');

%% Pop up
fprintf(t,'popup("Hey Mr. Copot", title="Popup#1",blocking=True)\n');

%% Speed (slider) ?
t.fprintf('socket_send_string("set speed")\n');
t.fprintf('socket_send_string(0.5)\n');
t.fprintf('socket_send_byte(10)\n');

