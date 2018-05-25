classdef GUI < handle
    %GUI Summary of this class goes here
    %   Detailed explanation goes here
    properties
        tDisp = 20;
        aTime
        aDist
        aSpeed
        fig
        axC
        axD
        pD
        axS
        pS
    end
    properties (Hidden = true)
        hReference
        hDist
        hLastDist
        hSF
        hTargetPose
        hState
        hHeight
    end
    
    methods
        function obj = GUI(varargin)
            p=inputParser;
            p.addParameter('ControlPanel',false,@islogical);
            p.addParameter('LiveGraphDist',false,@islogical);
            p.addParameter('LiveGraphSpeed',false,@islogical);
            p.parse(varargin{:});
            
            if p.Results.ControlPanel || p.Results.LiveGraphDist || p.Results.LiveGraphSpeed
                obj.fig =figure('Name','GUI');
            end
            
            if p.Results.ControlPanel
                xDes =20;
                xVal =120;
                wDes = xVal-xDes;
                xUni =170;
                wVal =xUni-(xDes+wDes);
                wUni = 20;
                
                obj.axC = subplot(2,2,[1,4]);
                obj.axC.Units='pixels';
                obj.axC.Visible='off';
                % description
                uicontrol('Style','text','Position',[xDes 300 wDes 20],...
                    'HorizontalAlignment','left','String','Reference:');
                uicontrol('Style','text','Position',[xDes 280 wDes 20],...
                    'HorizontalAlignment','left','String','Current Distance:');
                uicontrol('Style','text','Position',[xDes 260 wDes 20],...
                    'HorizontalAlignment','left','String','Last Distance:');
                uicontrol('Style','text','Position',[xDes 240 wDes 20],...
                    'HorizontalAlignment','left','String','SF:');
                uicontrol('Style','text','Position',[xDes 220 wDes 20],...
                    'HorizontalAlignment','left','String','TargetPose:');
                uicontrol('Style','text','Position',[xDes 200 wDes 20],...
                    'HorizontalAlignment','left','String','State:');
                uicontrol('Style','text','Position',[xDes 180 wDes 20],...
                    'HorizontalAlignment','left','String','Height:');
                % values
                obj.hReference =uicontrol('Style','text','Position',[xVal 300 wVal 20],...
                    'HorizontalAlignment','left','String','');
                obj.hDist =uicontrol('Style','text','Position',[xVal 280 wVal 20],...
                    'HorizontalAlignment','left','String','');
                obj.hLastDist =uicontrol('Style','text','Position',[xVal 260 wVal 20],...
                    'HorizontalAlignment','left','String','');
                obj.hSF=uicontrol('Style','text','Position',[xVal 240 wVal 20],...
                    'HorizontalAlignment','left','String','');
                obj.hTargetPose=uicontrol('Style','text','Position',[xVal 220 wVal 20],...
                    'HorizontalAlignment','left','String','');
                obj.hState=uicontrol('Style','text','Position',[xVal 200 150 20],...
                    'HorizontalAlignment','left','String','');
                obj.hHeight=uicontrol('Style','text','Position',[xVal 180 wVal 20],...
                    'HorizontalAlignment','left','String','');
                % units
                uicontrol('Style','text','Position',[xUni 280 wUni 20],...
                    'HorizontalAlignment','left','String','m');
                uicontrol('Style','text','Position',[xUni 260 wUni 20],...
                    'HorizontalAlignment','left','String','m');
                uicontrol('Style','text','Position',[xUni 180 wUni 20],...
                    'HorizontalAlignment','left','String','m');
                % graph
                obj.aTime = 0;
                obj.aDist = Inf;
                obj.aSpeed = Inf;
                
            end
            
            if p.Results.LiveGraphDist
                obj.axD = subplot(2,3,[2 3]);
                obj.pD=plot(obj.axD,obj.aTime,obj.aDist);
                title('Distance'); grid on
                xlabel('Time[s]')
                ylabel('Distance [m]')
                obj.axD.YLimMode= 'manual';
                obj.axD.YLim= [0 2.5];
                
            end
            
            if p.Results.LiveGraphSpeed
                obj.axS = subplot(2,3,[5 6]);
                obj.pS=plot(obj.axS,obj.aTime,obj.aSpeed);
                title('Speedfactor'); grid on
                xlabel('Time[s]')
                ylabel('Speedfactor []')
                obj.axS.YLimMode= 'manual';
                obj.axS.YLim= [0 1.1];
            end
        end
        function setValues(obj,varargin)
            p = inputParser;
            p.addRequired('obj');
            p.addParameter('Reference','');
            p.addParameter('Dist','');
            p.addParameter('LastDist','');
            p.addParameter('SF','');
            p.addParameter('TargetPose','');
            p.addParameter('State','');
            p.addParameter('Height','');
            p.addParameter('Time',[]);
            p.parse(obj,varargin{:});
            
            if ~strcmp('',p.Results.Reference)
                obj.hReference.String = num2str(p.Results.Reference);
            end
            if ~strcmp('',p.Results.Dist)
                obj.hDist.String = num2str(p.Results.Dist);
            end
            if ~strcmp('',p.Results.LastDist)
                obj.hLastDist.String = num2str(p.Results.LastDist);
            end
            if ~strcmp('',p.Results.SF)
                obj.hSF.String = num2str(p.Results.SF);
            end
            if ~strcmp('',p.Results.TargetPose)
                obj.hTargetPose.String = num2str(p.Results.TargetPose);
            end
            if ~strcmp('',p.Results.State)
                if p.Results.State == 0
                    text = 'Stopped';
                elseif p.Results.State == 1
                    text = 'Stopped (h)';
                elseif p.Results.State == 2
                    text = 'Moving';
                else
                    text='';
                
                end
                obj.hState.String = num2str(text);
            end
            if ~strcmp('',p.Results.Height)
                obj.hHeight.String = num2str(p.Results.Height);
            end
            
            if ~isempty(p.Results.Time)
                obj.aTime(length(obj.aTime)+1)=p.Results.Time;
                if ~strcmp('',p.Results.Dist)
                    obj.aDist(length(obj.aDist)+1)=p.Results.Dist;
                    obj.pD.XData = obj.aTime;
                    obj.pD.YData = obj.aDist;
                    obj.axD.XLim= [max(obj.aTime(end)-obj.tDisp,0) max(obj.aTime(end),obj.tDisp)];
                end
                if ~strcmp('',p.Results.SF)
                    obj.aSpeed(length(obj.aSpeed)+1)=p.Results.SF;
                    obj.pS.XData = obj.aTime;
                    obj.pS.YData = obj.aSpeed;
                    obj.axS.XLim= [max(obj.aTime(end)-obj.tDisp,0) max(obj.aTime(end),obj.tDisp)];
                end
            end
            
            drawnow
        end
        
    end
end

