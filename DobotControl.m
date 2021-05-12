classdef DobotControl < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        dobot;
        rackState = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        
        rack1Pos = {[0.15, 0.06, 0.03], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]};
        rack1Pos1 = [0.15, 0.06, 0.03];
        rack2Pos1 = [0.15, -0.06, 0.03];
        rack2Pos = {[0.15, -0.06, 0.03], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]};
        
        gripper = false;
        
    end
    
    methods
        %% To Add

        % Test moving tube function // hard code rack position
        % Calibrate Racks
        
        % Test move to functions
        % Test rack state functions
        % Test Estop
        % TEST GRIPPER
        
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
        
        %% Resume
        function ResumeDobot(self)
            self.dobot.ResumeRobot();
        end
        
        %% Gripper
        function OpenGripperDobot(self)
            self.dobot.PublishGripperState(true, true);
        end
        
        %% Gripper
        function CloseGripperDobot(self)
            self.dobot.PublishGripperState(true, false);
        end
        
        %% Cartesian Based Jogging
        function JogDobotCartesian(self, axis, direction, distance)
            
            targetEndEffector = self.dobot.GetCurrentEndEffectorState();
            
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
            
            self.MoveToCartesianPoint(targetEndEffector);
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
            
            state = 0;
            while state == 0
                self.dobot.PublishTargetJoint(targetJointState);
                state = self.CheckCurrentPosition(targetEndEffector);
                pause(0.3);
            end
        end
      
        %% Move to Cartesian Point
        function MoveToCartesianPoint(self, targetEndEffector)
            
            state = 0;
            endEffectorRotation = [0,0,0];
            
            while state == 0
                self.dobot.PublishEndEffectorPose(targetEndEffector, endEffectorRotation);
                currentEndEffector = self.dobot.GetCurrentEndEffectorState();
                
                dis = sqrt((currentEndEffector(1) - cartesianPosition(1))^2 + (currentEndEffector(2) - cartesianPosition(2))^2 + (currentEndEffector(3) - cartesianPosition(3))^2);
                
                if dis <= 0.005
                    state = 1;
                end
                
                pause(0.3);
            end
        end
        
        %% Move to Joint State
        function MoveToJointState(self, targetJointState)
            
            state = 0;
            
            while state == 0
                self.dobot.PublishTargetJoint(targetJointState);
                currentJointState = self.dobot.GetJointStates();
                
                dif1 = abs(targetJointState(1) - currentJointState(1));
                dif2 = abs(targetJointState(2) - currentJointState(2));
                dif3 = abs(targetJointState(3) - currentJointState(3));
                dif4 = abs(targetJointState(4) - currentJointState(4));
                
                if dif1 <= 0.1
                    if dif2 <= 0.1
                        if dif3 <= 0.1
                            if dif4 <= 0.1
                                state = 1;
                            end
                        end
                    end
                end
                
                pause(0.3);
            end
        end
        
        %% Get End Effector Position (Cartesian)
        function endEffector = GetEndEffectorPosition(self)
            currentEndEffector = self.dobot.GetCurrentEndEffectorState();
            endEffector = currentEndEffector;
        end
        
        %% Get Joint States
        function jointStates = GetJointStates(self)
            currentJointState = self.dobot.GetCurrentJointState();
            jointStates = currentJointState;
        end
        
        %% Set Rack State
        function SetRackState(self, rack, position, state)
            switch rack
                case '1'
                    rackState(position - 1) = state;
                case '2'
                    rackState(6 + position - 1) = state;
            end
        end
        
        %% Get Rack State
        function rackState = GetRackState(self)
            rackState =  self.rackState;
        end
        
        %% Move Test Tube
        function PeformTestTubeMove(self, fromRack, fromRackPos, toRack, toRackPose)
            
            liftOffset = [0, 0, 0.1];
            
            % Move to Test Tube position
            switch fromRack
                case '1'
                    %targetEndEffector = self.rack1Pos(fromRackPos);
                    targetEndEffector = self.rack1Pos1;
                case '2'
                    targetEndEffector = self.rack2Pos(fromRackPos);
            end
            
            self.MoveToCartesianPoint(targetEndEffector);
            
            % Gripper
            self.dobot.PublishToolState(true);
            
            % Lift
            self.MoveToCartesianPoint(targetEndEffector + liftOffset);
            
            % Move to Drop position
            switch toRack
                case '1'
                    targetEndEffector = self.rack1Pos(toRackPos);
                case '2'
                    %targetEndEffector = self.rack2Pos(toRackPos);
                    targetEndEffector = self.rack2Pos1;
            end
            
            self.MoveToCartesianPoint(targetEndEffector + liftOffset);
            
            % Move down
            self.MoveToCartesianPoint(targetEndEffector);
            
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

