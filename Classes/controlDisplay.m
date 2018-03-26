classdef controlDisplay < handle
    %display Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        hReference
        hDist
        hLastDist
        hSpeedfactor
        hTargetPose
        hState
    end
    
    methods
        function obj = controlDisplay()
            xDes =50;
            xVal =150;
            wDes = xVal-xDes;
            xUni =200;
            wVal =xUni-(xDes+wDes);
            wUni = 50;

            
            f =figure('Visible','off');
            axes('Units','pixels','Visible','off');
            % description
            uicontrol('Style','text','Position',[xDes 300 wDes 20],...
                'HorizontalAlignment','left','String','Reference:');
            uicontrol('Style','text','Position',[xDes 280 wDes 20],...
                'HorizontalAlignment','left','String','Current Distance:');
            uicontrol('Style','text','Position',[xDes 260 wDes 20],...
                'HorizontalAlignment','left','String','Last Distance:');
            uicontrol('Style','text','Position',[xDes 240 wDes 20],...
                'HorizontalAlignment','left','String','Speedfactor:');
            uicontrol('Style','text','Position',[xDes 220 wDes 20],...
                'HorizontalAlignment','left','String','TargetPose:');
            uicontrol('Style','text','Position',[xDes 200 wDes 20],...
                'HorizontalAlignment','left','String','State:');
            % values
            obj.hReference =uicontrol('Style','text','Position',[xVal 300 wVal 20],...
                'HorizontalAlignment','left','String','');
            obj.hDist =uicontrol('Style','text','Position',[xVal 280 wVal 20],...
                'HorizontalAlignment','left','String','');
            obj.hLastDist =uicontrol('Style','text','Position',[xVal 260 wVal 20],...
                'HorizontalAlignment','left','String','');
            obj.hSpeedfactor=uicontrol('Style','text','Position',[xVal 240 wVal 20],...
                'HorizontalAlignment','left','String','');
            obj.hTargetPose=uicontrol('Style','text','Position',[xVal 220 wVal 20],...
                'HorizontalAlignment','left','String','');
            obj.hState=uicontrol('Style','text','Position',[xVal 200 150 20],...
                'HorizontalAlignment','left','String','');
            % units
            uicontrol('Style','text','Position',[xUni 280 wUni 20],...
                'HorizontalAlignment','left','String','m');
            uicontrol('Style','text','Position',[xUni 260 wUni 20],...
                'HorizontalAlignment','left','String','m');
            
            f.Visible = 'on';
        end
        function setValues(obj,varargin)
            p = inputParser;
            p.addRequired('obj');
            p.addParameter('Reference','');
            p.addParameter('Dist','');
            p.addParameter('LastDist','');
            p.addParameter('Speedfactor','');
            p.addParameter('TargetPose','');
            p.addParameter('State','');
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
            if ~strcmp('',p.Results.Speedfactor)
                obj.hSpeedfactor.String = num2str(p.Results.Speedfactor);
            end
            if ~strcmp('',p.Results.TargetPose)
                obj.hTargetPose.String = num2str(p.Results.TargetPose);
            end
            if ~strcmp('',p.Results.State)
                if p.Results.State == 0
                    text = 'Stopped';
                elseif p.Results.State == 1
                    text = 'Next Target';
                elseif p.Results.State == 2
                    text = 'Moving';
                end
                obj.hState.String = num2str(text);
            end
            drawnow
        end
        
    end
end

