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
        % Joint limits
        % Rack Arrangement
        
        
        %% Constructor
        function self = DobotControl()
            self.dobot = DobotMagician();
            
            disp('ROBOT CREATED');
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

            targetEndEffector = currentEndEffector;
            
            if direction == 'Neg'
                distance = distance * -1;
            end
                        
            switch axis
                case 'X'
                    targetEndEffector(1) = targetEndEffector(1) + distance;
                case 'Y'
                    targetEndEffector(2) = targetEndEffector(2) + distance;
                case 'Z'
                    targetEndEffector(3) = targetEndEffector(3) + distance;
            end
            
            self.dobot.PublishEndEffectorPose(targetEndEffector, endEffectorRotation);
            
        end
        
        %% Joint Based Jogging
        function JogDobotJoint(self, joint, direction, distance)
            
            currentJointState = self.dobot.GetCurrentJointState();
            targetJointState = currentJointState;
            
            if direction == 'Neg'
                distance = distance * -1;
            end
            
            switch joint
                case '1'
                    targetJointState(1) = currentJointState(1) + distance;
                case '2'
                    targetJointState(2) = currentJointState(2) + distance;
                case '3'
                    targetJointState(3) = currentJointState(3) + distance;
                case '4'
                    targetJointState(4) = currentJointState(4) + distance;
            end
            
            self.dobot.PublishTargetJoint(targetJointState);
        end
        
        %% Get End Effector Position and Joint States
        
        function endEffectorAndJointStates = GetEndEffectorAndJointStates()
            
            currentEndEffector = self.dobot.GetCurrentEndEffectorState();
            currentJointState = self.dobot.GetCurrentJointState();
            
            endEffectorAndJointStates = [currentEndEffector, currentJointState];
        end
        
        %% Set Rack State
        function SetRackState(self, rack, position, state)
            
            switch rack
                case '1'
                    rack1(position - 1) = state;
                case '2'
                    rack2(position - 1) = state;
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

