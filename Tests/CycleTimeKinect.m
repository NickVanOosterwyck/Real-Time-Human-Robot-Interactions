%% init
cam=kinectcore('real');
cam.connect();

ct_ptCloud =0;
ct_skeleton =0;

%% test
p=0;

pause(5)
n=20;
t1=zeros(1,n);
t2=zeros(1,n);
for i=1:n
    tic;
    [ptCloud] = cam.getPointCloud('Filtered');
    t1(i)=toc;
    tic;
    [bodies] = cam.getSkeleton();
    t2(i)=toc;
end
ct_ptCloud(p+1)=mean(t1)
ct_skeleton(p+1)=mean(t2)

%%
%plot
plot(0:length(ct_ptCloud)-1,ct_ptCloud,'o',0:length(ct_skeleton)-1,ct_skeleton,'o','Linewidth',2)
legend('cycle time pointcloud','cycle time skeleton')
axis([-0.1 length(ct_ptCloud)-1+0.1 0 max([ct_ptCloud ct_skeleton])+0.05])
set(gca,'XTick',0:length(ct_ptCloud)-1);
xlabel('number of detected people')
ylabel('cycle time [s]')
grid on

fig = gcf;
fig.PaperPositionMode = 'auto';
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_pos(3) fig_pos(4)];

