%% init
rob=ur10core('real');
rob.connect();

Joint=6;

%% home
a=1.4; v=0.4; t=0; r=0;
rob.movej([0 -90 -90 -180 -90 0],a,v,t,r);
rob.setSpeedFactor(1);

%% speed slider
a=1.4; v=0.8; t=0; r=0;
PosS=0; TimeS=0; SpeedS=0;
t_end=5;
k=1;
tic
pause(1)
rob.movej([0 -90 -90 -180 -90 90],a,v,t,r);
while toc<t_end
    t1=toc;
    if t1>3
        rob.setSpeedFactor(0);
    end
    Positions = rob.getJointPositions();
    PosS(k)= Positions(Joint);
    TimeS(k)=toc;
    SpeedS(k) = rob.getTCPSpeed();
    k=k+1;
end
%% stopj
a=1.4; v=0.8; t=0; r=0;
PosM=0; TimeM=0; SpeedM=0;
t_end=5;
k=1;
tic
pause(1)
rob.movej([0 -90 -90 -180 -90 90],a,v,t,r);
while toc<t_end
    t1=toc;
    if t1>3
        rob.stopj(10);
    end
    Positions = rob.getJointPositions();
    PosM(k)= Positions(Joint);
    TimeM(k)=toc;
    SpeedM(k) = rob.getTCPSpeed();
    k=k+1;
end

%% plot
plot(TimeM,PosM,TimeS,PosS)
xlabel('Time [s]')
ylabel(['Joint ' num2str(Joint) ' Position [°]'])
axis([0 5 0 100])
legend('Send \it stopj','Speed slider to 0')
grid on
fig = gcf;
fig.PaperPositionMode = 'auto';
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_pos(3) fig_pos(4)];

figure
plot(TimeM,SpeedM,TimeS,SpeedS)
xlabel('Time [s]')
ylabel('TCP Speed [m/s]')
legend('Send \it stopj','Speed slider to 0')
axis([0 5 0 0.1])
grid on
fig = gcf;
fig.PaperPositionMode = 'auto';
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_pos(3) fig_pos(4)];