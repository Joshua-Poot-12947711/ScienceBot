classdef DobotControl < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        dobot;
        rack1 = [0, 0, 0, 0, 0, 0];
        rack2 = [0, 0, 0, 0, 0, 0];
        
        origin = [0, 0, 0];
        
        rack1Pos = {[0.15, 0.06, 0.03], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]};
        rack1Pos1 = [0.15, 0.06, 0.03];
        rack2Pos1 = [0.15, -0.06, 0.03];
        rack2Pos = {[0.15, -0.06, 0.03], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]};
        
        gripper = false;
        
    end
    
    methods
        %% To Add
        % Joint limits function?
        % Test position check function
        % Test moving tube function // hard code rack position
        % Calibrate Racks
        
        % Position is printing in radians & rounding wrong
        
        
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
        
        %% Resume
        function ResumeDobot(self)
            self.dobot.ResumeRobot();
        end
        
                %% Gripper
        function OpenGripperDobot(self)
            
            self.dobot.PublishToolState(true);
            
        end
        
                        %% Gripper
        function CloseGripperDobot(self)
            
            self.dobot.PublishToolState(false);
            
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
            
            
            state = 0;
            
            while state == 0
                state = self.CheckCurrentPosition(targetEndEffector);
                pause(0.3);
            end
            
        end
        
        %% Check Position is within Robot Limits
        function achievable = CheckWithinLimit(self, cartesianPosition)
            
            achievable = 1;
            
            % Find maximum extension of arm? Check if point is within
            % radius?
            
        end
        
        %% Check Current Position of Dobot with Goal Position
        function achieved = CheckCurrentPosition(self, cartesianPosition)
            
            
            
            currentEndEffector = self.dobot.GetCurrentEndEffectorState();
            
             dis = sqrt((currentEndEffector(1) - cartesianPosition(1))^2 + (currentEndEffector(2) - cartesianPosition(2))^2 + (currentEndEffector(3) - cartesianPosition(3))^2)
             %norm(currentEndEffector - cartesianPosition)
             
             if dis <= 0.005
             achieved = 1
             return
             end
             
             achieved = 0 
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
        
        %% Get End Effector Position (Cartesian) and Joint States
        function endEffector = GetEndEffectorPosition(self)
            
            currentEndEffector = self.dobot.GetCurrentEndEffectorState();
            
            endEffector = currentEndEffector;
        end
        
        %% Get End Effector Position (Cartesian) and Joint States
        function jointStates = GetJointStates(self)
            
            currentJointState = self.dobot.GetCurrentJointState();
            
            jointStates = currentJointState;
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
        
        %% Move Test Tube
        function PeformTestTubeMove(self, fromRack, fromRackPos, toRack, toRackPose)
            
            liftOffset = [0, 0, 0.1];
            endEffectorRotation = [0,0,0];
            
            % Move to Test Tube position
            switch fromRack
                case '1'
                    %targetEndEffector = self.rack1Pos(fromRackPos);
                    targetEndEffector = self.rack1Pos1;
                case '2'
                    targetEndEffector = self.rack2Pos(fromRackPos);
            end
            
            %if self.CheckWithinLimit(targetEndEffector) ~= 1
            %    return;
            %end
            
            endEffectorRotation = [0,0,0];
            
            state = 0;
            
            while state == 0
                self.dobot.PublishEndEffectorPose(targetEndEffector, endEffectorRotation)
                state = self.CheckCurrentPosition(targetEndEffector);
                pause(0.3);
            end
            
            
            % Gripper
            self.dobot.PublishToolState(true);
            
            % Lift up
            self.dobot.PublishEndEffectorPose(targetEndEffector + liftOffset, endEffectorRotation);
            
            state = 0;
            
            while state == 0
                % Lift up
                self.dobot.PublishEndEffectorPose(targetEndEffector + liftOffset, endEffectorRotation);
                state = self.CheckCurrentPosition(targetEndEffector);
                pause(0.3);
            end
            
            % Move to Drop position
            switch toRack
                case '1'
                    targetEndEffector = self.rack1Pos(toRackPos);
                case '2'
                    %targetEndEffector = self.rack2Pos(toRackPos);
                    targetEndEffector = self.rack2Pos1;
            end
            
            %if self.CheckWithinLimit(targetEndEffector) ~= 1
            %    return;
            %end
            
            
            disp('Stoppy');
            state = 0;
            
            while state == 0
                self.dobot.PublishEndEffectorPose(targetEndEffector + liftOffset, endEffectorRotation)
                state = self.CheckCurrentPosition(targetEndEffector);
                pause(0.3);
            end
                        
            % Move down
            
            disp('Downy');
            state = 0;
            
            while state == 0
                self.dobot.PublishEndEffectorPose(targetEndEffector, endEffectorRotation);
                state = self.CheckCurrentPosition(targetEndEffector);
                pause(0.3);
            end
            
            % Release Gripper
            self.dobot.PublishToolState(false);
            
        end
        
        %% Calibrates Tube Positions
        function CalibrateTestTubePositions(self)
            
            rack1Pos(1) = [0.15, 0.06, 0.03];
            rack2Pos(1) = [0.15, -0.06, 0.03];
            
        end
        
        %% Function
        %         %% Function
        %         function outputArg = method1(obj,inputArg)
        %             %METHOD1 Summary of this method goes here
        %             %   Detailed explanation goes here
        %             outputArg = obj.Property1 + inputArg;
        %         end
        
        
    end
end

