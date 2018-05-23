%% init
rob=ur10core('real');
rob.connect();
rob2=ur10core('vrep');
rob2.connect();

Joint=6;

%% home
a=10; v=0.4; t=0; r=0;
rob.setSpeedFactor(1);
rob2.setSpeedFactor(1);
rob.movej([0 -90 -90 -180 -90 0],a,v,t,r)
rob2.movej([0 -90 -90 -180 -90 0],a,v,t,r)

%% test
a=10; v=0.4; t=0; r=0;
t_end=10;
k=1;
PosM=0; TimeM=0; SpeedM=0;
PosS=0; TimeS=0; SpeedS=0;
tic
pause(1)
rob.movej([0 -90 -90 -180 -90 180],a,v,t,r);
rob2.movej([0 -90 -90 -180 -90 180],a,v,t,r);
while toc<t_end
    Positions = rob.getJointPositions();
    PosM(k)= Positions(Joint);
    TimeM(k)=toc;
    SpeedM(k) = rob.getTCPSpeed();
    
    Positions2 = rob2.getJointPositions();
    PosS(k)= Positions2(Joint);
    TimeS(k)=toc;
    SpeedS(k) = rob2.getTCPSpeed();
    k=k+1;
end

%% plot
figure
plot(TimeM,PosM,TimeS,PosS)
xlabel('Time [s]')
ylabel(['Joint ' num2str(Joint) ' Position [°]'])
axis([0 10 0 220])
legend('Real Robot','Simulated Robot')
grid on
fig = gcf;
fig.PaperPositionMode = 'auto';
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_pos(3) fig_pos(4)];

figure
plot(TimeM,SpeedM,TimeS,SpeedS)
xlabel('Time [s]')
ylabel('TCP Speed [m/s]')
legend('Real Robot','Simulated Robot')
axis([0 10 0 0.06])
grid on
fig = gcf;
fig.PaperPositionMode = 'auto';
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_pos(3) fig_pos(4)];