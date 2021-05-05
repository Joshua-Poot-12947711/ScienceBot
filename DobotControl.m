classdef DobotControl < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        dobot;
        rack1 = [0, 0, 0, 0, 0, 0];
        rack2 = [0, 0, 0, 0, 0, 0];
        
    end
    
    methods
        %% To Add
        % Current End Effector Pose on GUI
        % Joint limits
        % Rack Arrangement
        
        
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
        end
        
        %% Cartesian Based Jogging
        function JogDobotCartesian(self, axis, direction, distance)
            
            currentEndEffector = self.dobot.GetCurrentEndEffectorState();
            endEffectorRotation = [0,0,0];
            
            if direction == 'Neg'
                distance = distance * -1;
            end
            
            switch axis
                case 'X'
                    targetEndEffector = currentEndEffector + [distance, 0, 0];
                    disp(targetEndEffector);
                case 'Y'
                    targetEndEffector = currentEndEffector + [0, distance, 0];
                case 'Z'
                    targetEndEffector = currentEndEffector + [0, 0, distance];
            end
            
            disp(targetEndEffector);
            self.dobot.PublishEndEffectorPose(targetEndEffector, endEffectorRotation);
        end
        
        %% Joint Based Jogging
        function JogDobotJoint(self, joint, direction, distance)
            
            currentJointState = self.dobot.GetCurrentJointState();
            
            if direction == 'Neg'
                distance = distance * -1;
            end
            
            switch joint
                case '1'
                    targetJointState = currentJointState + [distance, 0, 0, 0];
                case '2'
                    targetJointState = currentEndEffector + [0, distance, 0, 0];
                case '3'
                    targetJointState = currentEndEffector + [0, 0, distance, 0];
                case '4'
                    targetJointState = currentEndEffector + [0, 0, 0, distance];
            end
            
            disp(targetJointState);
            self.dobot.PublishTargetJoint(targetJointState);
        end
        
        %% Set Rack State
        function SetRackState(self, rack, position, state)
            
            switch rack
                case '1'
                    rack1[position - 1] = state;
                case '2'
                    rack2[position - 1] = state;
            end
        end
            
        %% Get Rack State
        function rackState = GetRackState(self)
            rackState =  [self.rack1, self.rack2];
        end
        
        %%
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

