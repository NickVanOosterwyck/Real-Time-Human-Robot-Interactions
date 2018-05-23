%% init
rob=ur10core('real');
rob.connect();

Joint=6;

%% home
a=1.4; v=0.4; t=0; r=0;
rob.setSpeedFactor(1);
rob.movej([0 -90 -90 -180 -90 0],a,v,t,r);

%% movej
a=1.4; v=0.4; t=0; r=0;
n=10;
t_end=10;
v_vec = linspace(v,v/10,n);
t_vec = linspace(1,t_end,10);
i=1;  k=1;
PosM=0; TimeM=0; SpeedM=0;
tic
while toc<t_end
    t1=toc;
    if t1>t_vec(i)
        rob.movej([0 -90 -90 -180 -90 180],a,v_vec(i),t,r);
        i=i+1;
    end
    Positions = rob.getJointPositions();
    PosM(k)= Positions(Joint);
    TimeM(k)=toc;
    SpeedM(k) = rob.getTCPSpeed();
    k=k+1;
end
%% speed slider
a=1.4; v=0.4; t=0; r=0;
n=10;
t_end=10;
v_vec = linspace(1,0.1,n);
t_vec = linspace(1,t_end,10);
i=1;  k=1;
PosS=0; TimeS=0; SpeedS=0;
tic
pause(1)
rob.movej([0 -90 -90 -180 -90 180],a,v,t,r);
while toc<t_end
    t1=toc;
    if t1>t_vec(i)
        rob.setSpeedFactor(v_vec(i));
        i=i+1;
    end
    Positions = rob.getJointPositions();
    PosS(k)= Positions(Joint);
    TimeS(k)=toc;
    SpeedS(k) = rob.getTCPSpeed();
    k=k+1;
end


%% plot
plot(TimeM,PosM,TimeS,PosS)
xlabel('Time [s]')
ylabel(['Joint ' num2str(Joint) ' Position [°]'])
axis([0 10 0 180])
legend('Resend \it movej','Change speed slider')
grid on
fig = gcf;
fig.PaperPositionMode = 'auto';
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_pos(3) fig_pos(4)];

figure
plot(TimeM,SpeedM,TimeS,SpeedS)
xlabel('Time [s]')
ylabel('TCP Speed [m/s]')
legend('Resend \it movej','Change speed slider')
axis([0 10 0 0.07])
grid on
fig = gcf;
fig.PaperPositionMode = 'auto';
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_pos(3) fig_pos(4)];
