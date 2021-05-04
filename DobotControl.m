classdef DobotControl < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        dobot;
        
    end
    
    methods
        %% Constructor
        function self = DobotControl()
            self.dobot = DobotMagician();
            
        end
        
        %% Home
        function HomeDobot(self)
            self.dobot.InitaliseRobot();
        end
        
        %% EStop
        function EStopDobot(self)
            self.dobot.EStopRobot();
            fprintf('COOL');
        end
        
        %% Cartesian Based Jogging
        function JogDobotCartesian(self, direction)

        end
        
        %% Joint Based Jogging
        function JogDobotJoint(self, direction)
            
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

