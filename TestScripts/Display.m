sub=rossubscriber('/ur_driver/URScript');

a=1.4; v=0.2; t=0; r=0;
q =[45 -110 -80 -170 -135 0];

msg=rosmessage('std_msgs/String');
q_str = ['[' num2str(q(1)) ',' num2str(q(2)) ',' num2str(q(3)) ',' num2str(q(4)) ',' num2str(q(5)) ',' num2str(q(6)) ']'];
msg.Data = ['movej(' q_str ',' num2str(a) ',' num2str(v) ',' num2str(t) ',' num2str(r) ')\n'];
send(scriptpub,msg);
