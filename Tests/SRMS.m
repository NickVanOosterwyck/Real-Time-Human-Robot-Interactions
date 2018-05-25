%% profile
syms x
f1=@(x) (2*(heaviside(x-0)-heaviside(x-2)))+...
    (((-0.25*x)+2.5)*(heaviside(x-2)-heaviside(x-4)))+...
    ((1.5)*(heaviside(x-4)-heaviside(x-6)))+...
    (((-0.5*x)+4.5)*(heaviside(x-6)-heaviside(x-8)))+...
    ((0.5)*(heaviside(x-8)-heaviside(x-10)))+...
    (((0.5*x)-4.5)*(heaviside(x-10)-heaviside(x-13)))+...
    ((2)*(heaviside(x-13)-heaviside(x-15)));
fplot(f1,[-1, 16]); clc;

%% Create & Connect
CameraType = 'vrep';    % vrep or real
RobotType = 'vrep';     % vrep or real

ctrl = controller(CameraType,RobotType);
ctrl.connect();

%% home
a=1.4; v=0.4; t=0; r=0;
ctrl.rob.setSpeedFactor(1);
ctrl.rob.movej([0 -90 -90 -180 -90 0],a,v,t,r);

%% test
Range = 0.05;
th_dist = 0.1;
a=10; v=0.2; t=0; r=0;
rStop = 1.3;

state=2;SF=1;
k=1;
Distance=0; Time=0; TCPspeed=0;
tic
ctrl.rob.movej([180 -90 -90 -180 -90 0],a,v,t,r);
while toc<15
    time=toc;
    Dist=f1(time);
    % determine speed
    if Dist<rStop && abs(LastDist-Dist)>th_dist
        LastDist=Dist;
        if state~=2
            ctrl.rob.stopj(10)
            state=2;
            SF=0;
        end
    elseif Dist>rStop
        if state~=1
        ctrl.rob.movej([180 -90 -90 -180 -90 0],a,v,t,r);
        state=1;
        SF=1;
        end
    end
    ctrl.rob.setSpeedFactor(SF);
    
    Distance(k)=Dist;
    Time(k)=time;
    %TCPspeed(k)=ctrl.rob.getTCPSpeed();
    TCPspeed(k)=SF*0.13;
    k=k+1;
end
disp('End of loop reached')

%% plot
subplot(2,1,1)
plot(Time,Distance)
xlabel('Time [s]')
ylabel('Distance [m]')
axis([0 15 0 2.2])
grid on; hold on;
plot([0 15],[1.3 1.3],'--r');
plot([6.4 6.4],[0 1.3],'--k');
plot([11.6 11.6],[0 1.3],'--k');
text(7,1.5,'Stop Distance')
subplot(2,1,2)
plot(Time,TCPspeed)
xlabel('Time [s]')
ylabel('TCP Speed [m/s]')
axis([0 15 0 0.3])
grid on; hold on;
plot([6.4 6.4],[0 2.2],'--k');
plot([11.6 11.6],[0 2.2],'--k');
fig = gcf;
fig.PaperPositionMode = 'auto';
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_pos(3) fig_pos(4)];
