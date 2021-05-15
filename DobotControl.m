classdef DobotControl < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        dobot;
        rackState = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        
        rack1Pos = {[0.14, 0.13, 0.07], [0.14, 0.104, 0.07], [0.14, 0.075, 0.07], [0.14, 0.055, 0.07], [0.14, 0.035, 0.07], [0.14, 0.015, 0.07]};
        rack2Pos = {[0.14, -0.005, 0.07], [0.14, -0.025, 0.07], [0.14, -0.045, 0.07], [0.14, -0.064, 0.07], [0.14, -0.09, 0.07], [0.14, -0.12, 0.07]};
        
        eStopped = 0;
        resumed = 1;
    end
    
    methods
        %% To Add
        % Calibrate Racks
        
        % Test Estop
        % Rack update ting
        % Fix compressor brr
        
        
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
            
            if self.eStopped == 0
                self.resumed = 0;
                self.eStopped = 1;
                
            elseif self.eStopped == 1
                self.eStopped = 0;
            end
            
            
            
        end
        
        %% Resume
        function ResumeDobot(self)
            
            self.resumed = 1;
            
        end
        
        %% Gripper
        function OpenGripperDobot(self)
            self.dobot.PublishGripperState(1, 0);
        end
        
        %% Gripper
        function CloseGripperDobot(self)
            self.dobot.PublishGripperState(1, 1);
        end
        
        %% Compressor Off
        function CompressorOff(self)
            self.dobot.PublishGripperState(0, 0);
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
            
            self.MoveToJointState(targetJointState);
        end
        
        %% Move to Cartesian Point
        function MoveToCartesianPoint(self, targetEndEffector)
            
            state = 0;
            endEffectorRotation = [0,0,0];
            
            while state == 0
                
                if self.eStopped == 1 %&& self.resumed == 0
                    pause(0.3);
                end
                
                
                if self.eStopped == 0 %&& self.resumed == 1
                    currentEndEffector = self.dobot.GetCurrentEndEffectorState();
                    self.dobot.PublishEndEffectorPose(targetEndEffector, endEffectorRotation);
                    
                    dis = sqrt((currentEndEffector(1) - targetEndEffector(1))^2 + (currentEndEffector(2) - targetEndEffector(2))^2 + (currentEndEffector(3) - targetEndEffector(3))^2);
                    
                    if dis <= 0.005
                        state = 1;
                    end
                    
                    pause(0.3);
                end

            end
            
        end
        
        %% Move to Joint State
        function MoveToJointState(self, targetJointState)
            
            state = 0;
            
            while state == 0
                self.dobot.PublishTargetJoint(targetJointState);
                currentJointState = self.dobot.GetCurrentJointState();
                
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
                    self.rackState(position) = state;
                case '2'
                    self.rackState(6 + position) = state;
            end
        end
        
        %% Get Rack State
        function rackState = GetRackState(self)
            rackState =  self.rackState;
        end
        
        %% Move Test Tube
        function PeformTestTubeMove(self, fromRack, fromRackPos, toRack, toRackPos)
            
            liftOffset = [0, 0, 0.08];
            transportOffset = [0.03, 0, 0];
            
            self.OpenGripperDobot();
            
            % Move to Test Tube position
            disp('Moving to tube.');
            switch fromRack
                case '1'
                    targetEndEffector = self.rack1Pos{fromRackPos};
                case '2'
                    targetEndEffector = self.rack2Pos{fromRackPos};
            end
            
            self.MoveToCartesianPoint(targetEndEffector + liftOffset);
            
            % Move down
            disp('Moving down.');
            self.MoveToCartesianPoint(targetEndEffector);
            
            
            % Gripper
            disp('Gripping.');
            self.CloseGripperDobot();
            pause(1);
            
            % Lift
            disp('Lifting.');
            self.MoveToCartesianPoint(targetEndEffector + liftOffset);
            
            % Move out
            self.MoveToCartesianPoint(targetEndEffector + liftOffset + transportOffset);
            
            % Move to Drop position
            disp('Moving to drop position.');
            switch toRack
                case '1'
                    targetEndEffector = self.rack1Pos{toRackPos};
                case '2'
                    targetEndEffector = self.rack2Pos{toRackPos};
            end
            
            self.MoveToCartesianPoint(targetEndEffector + liftOffset + transportOffset);
            
            % Move in
            self.MoveToCartesianPoint(targetEndEffector + liftOffset);
            
            % Move down
            disp('Moving down.');
            self.MoveToCartesianPoint(targetEndEffector);
            
            % Release Gripper
            disp('Releasing tube.');
            self.OpenGripperDobot();
            
        end
        
        %% Calibrates Tube Positions
        function CalibrateTestTubePositions(self)
            %% Calibrating the camera
            % setting the folder the images are in. Make sure the current folder in matlab is the folder before the folder below.
            imageFolder = './CalibrationImages';
            
            % Square Size is the size of each square on the checkerboard in mm
            squareSize = 30;
            % converting square size to m
            squareSize = squareSize/1000;
            
            % get the file path for the calibration images
            imageArray = dir(imageFolder);
            imageArray = {imageArray(~[imageArray.isdir]).name};
            for i = 1:length(imageArray)
                imageArray{i} = [imageFolder filesep imageArray{i}];
            end
            
            % find checkerboard points
            [points, boardSize, imagesUsed] = detectCheckerboardPoints(imageArray);
            if(imagesUsed == 0)
                error("No calibration images are found in the folder CalibrationImages");
            end
            
            % generate an ideal checkerboard to compare calibration images with
            worldPoints = generateCheckerboardPoints(boardSize, squareSize);
            
            % find camera parameters
            cameraParams = estimateCameraParameters(points, worldPoints, 'WorldUnits', 'm', 'NumRadialDistortionCoefficients', 2, ...
                'EstimateTangentialDistortion', true);
            %% Getting AR Tag poses and put them into class variables
            %get ar tag information from ar_track_alvar_msgs/AlvarMarkers
            ARTagSub = rossubscriber('/tags','/geometry_msgs/PoseArray');
            tagMsg = recieve(ARTagSub);
            transEndEff = self.dobot.GetCurrentEndEffectorState();
            
            trEndEff = eye(4);
            trEndEff(13) = transEndEff(1);
            trEndEff(14) = transEndEff(2);
            trEndEff(15) = transEndEff(3);
            
            % Transform for Camera to AR Tag at base - assuming 0 is the base AR Tag
            %offset of the tag to the base in m
            offset = [0.1, 0.1];
            
            transCamDobotBase = [tagMsg.Poses(1).Pose.Pose.Position.X, tagMsg.Poses(1).Pose.Pose.Position.Y,...
                tagMsg.Poses(1).Pose.Pose.Position.Z]
            rotCamDobotBase = quat2rotm(tagMsg.Poses(1).Pose.Pose.Orientation.W, ...
                tagMsg.Poses(1).Pose.Pose.Orientation.X, ...
                tagMsg.Poses(1).Pose.Pose.Orientation.Y, ...
                tagMsg.Poses(1).Pose.Pose.Orientation.Z);
            
            trCamDobotBase = rt2tr(rotCamDobotBase, transCamDobotBase);
            trCamEndEff = trCamDobotBase + trEndEff;
            
            % Transform for Camera to Object - assuming 1 is object AR Tag
            % cell array for ar tag transforms
            ARTagPoses = cell(size(tagMsg.markers,2),1);
            for i = 2:1:size(tagMsg.Poses)
                trans_cam_object = [tagMsg.Poses(i).Pose.Pose.Position.X, tagMsg.Poses(i).Pose.Pose.Position.Y,...
                    tagMsg.Poses(i).Pose.Pose.Position.Z]
                rot_cam_object = quat2rotm(tagMsg.Poses(i).Pose.Pose.Orientation.W, ...
                    tagMsg.Poses(i).Pose.Pose.Orientation.X, ...
                    tagMsg.Poses(i).Pose.Pose.Orientation.Y, ...
                    tagMsg.Poses(i).Pose.Pose.Orientation.Z);
                ARTagPoses{i} = rt2tr(rot_cam_object, trans_cam_object);
            end
            
            % Calculate transforms relative to the robot end eff.
            % Rack 1 is tag 1->6, Rack 2 is tag 7->12
            for i = 1:1:6
                relativeTr = inv(ARTagPoses{i})*trCamEndEff;
                self.rack1Pos{i} = [relativeTr(13), relativeTr(14), relativeTr(15)];
            end
            for i = 7:1:12
                relativeTr = inv(ARTagPoses{i})*trCamEndEff;
                self.rack2Pos{i} = [relativeTr(13), relativeTr(14), relativeTr(15)];
            end
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

