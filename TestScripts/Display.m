Dist = 20;
LastDist = 8;
PoseNr = 3;

f = figure('Visible','off');
ax = axes('Units','pixels','Visible','off');
txt = uicontrol('Style','text',...
    'Position',[50 300 200 20],...
    'HorizontalAlignment','left',...
    'String',['Current Distance = ' num2str(Dist) ' m']);
txt2 = uicontrol('Style','text',...
    'Position',[150 300 120 20],...
    'String',['Current Distance = ' num2str(Dist) ' m']);
f.Visible = 'on';

pause(2)
txt.String = 'test';
drawnow
