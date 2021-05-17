classdef DobotSimulation < handle

       properties(Access = public)
        %> Robot model
        model;
       end
       properties(Access = private)
        %> Workspace
        workspace = [-4 4 -4 4 -4 4];   
        
        %> Arm lengths for Dobot
        armLengths = [0.057, 0.135, 0.147, 0, 0];

       end
       
       methods(Access = public)
           %% Class Initialisation
           function self = DobotSimulation()
               %initialising the simulation
               self.getDobot();
               q = zeros(1,5);
               plot(self.model,q);
           end
           
           %% Get Dobot Parameters and create seriallink
           function getDobot(self)
               %create Dobot name after 1ms pause
               pause(0.001);
               name = ['Dobot_',datestr(now,'yyyy')];
               
               L1 = Link('d',0,'a',0.057,'alpha',0,'offset',pi/2,'qlim',[deg2rad(-135),deg2rad(135)]);
               L2 = Link('d',0,'a',0.135,'alpha',0,'offset',-pi/2,'qlim',[deg2rad(5),deg2rad(80)]);
               L3 = Link('d',0,'a',0.147,'alpha',0,'offset',0,'qlim',[deg2rad(15),deg2rad(170)]);
               L4 = Link('d',0,'a',0,'alpha',0,'offset',0,'qlim',[pi/2,pi/2]);
               L5 = Link('d',0,'a',0,'alpha',0,'offset',0,'qlim',[deg2rad(--85),deg2rad(85)]);
               
               self.model = SerialLink([L1 L2 L3 L4 L5], 'name', name);
           end
           
           %% Dobot Fkine
           function translation = DobotFkine(self, q)
               % the below is based on forward kinematic calculations done on the Dobot
               x = self.armLengths(2)*cos(q(1))*sin(q(2))+self.armLengths(3)*cos(q(1))*cos(q(3));
               y = self.armLengths(2)*sin(q(1))*sin(q(2))+self.armLengths(3)*sin(q(1))*sin(q(3));
               z = self.armLengths(2)*cos(q(2)) - self.armLengths(3)*sin(q(3))-self.armLengths(1);
               
               translation = [x, y, z];
           end
           %% Dobot Ikine
           function jointAngles = DobotIkine(self, translation)
               % the below is based on calculations we were given as part of the Dobot literature
               % let translation = [x, y, z]
               x = translation(1);
               y = translation(2);
               z = translation(3);
               
               length = sqrt(x^2+y^2);
               distance = sqrt(length^2+z^2);
               t1 = atan(z/l);
               t2 = acosd(((armLength(1)^2)+distance^2-(armLength(2))^2)/(2*armLength(1)*distance));
               alpha = t1 + t2;
               beta = acosd(((armLength(1)^2)+(armLength(2)^2)-distance^2)/(2*armLength(1)*armLength(2)));
               jointAngles = [atan(y/x), (pi/2 - alpha), (pi - beta - alpha)];
           end
           
                   %% Dobot Ikcon
        function [qstar, error, exitflag] = ikcon(self, robot, T, q0)
            
            % create output variables
            T_sz = size(T, 3);
            qstar = zeros(T_sz, 4);
            error = zeros(T_sz, 1);
            exitflag = zeros(T_sz, 1);
            
            problem.x0 = zeros(1, 4);
            problem.options = optimoptions('fmincon', ...
                'Algorithm', 'active-set', ...
                'Display', 'off'); % default options for ikcon
            
            if nargin > 2
                % user passed initial joint coordinates
                problem.x0 = q0;
            end
            
            % set the joint limit bounds (lb = lower bounds, ub = upper bounds)
            problem.lb = self.r_qlim(:,1);
            problem.ub = self.r_qlim(:,2);
            problem.solver = 'fmincon';
            
            reach = sum(abs([robot.a, robot.d]));
            omega = diag([1 1 1 3/reach]);
            
            for t = 1: T_sz
                problem.objective = ...
                    @(x) sumsqr(((T(:,:,t) \ robot.fkine(x)) - eye(4)) * omega);
                
                [q_t, err_t, ef_t] = fmincon(problem);
                
                qstar(t,:) = q_t;
                error(t) = err_t;
                exitflag(t) = ef_t;
                
                problem.x0 = q_t;
            end
        end
        
        function s = sumsqr(A)
            s = sum(A(:).^2);
        end
        
        %% Collision detection with a robot and a rectangular prism
        function result = IsCollision(self, robot, qMatrix, faces, vertex, faceNormals, returnOnceFound)
            if nargin < 6
                returnOnceFound = true;
            end
            result = false;
            
            for qIndex = 1:size(qMatrix,1)
                % Get the transform of every joint (i.e. start and end of every link)
                tr = GetLinkPoses(qMatrix(qIndex,:), robot);
                
                % Go through each link and also each triangle face
                for i = 1 : size(tr,3)-1
                    for faceIndex = 1:size(faces,1)
                        vertOnPlane = vertex(faces(faceIndex,1)',:);
                        [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)');
                        if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                            plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                            display('Intersection');
                            result = true;
                            if returnOnceFound
                                return
                            end
                        end
                    end
                end
            end
        end
       end
end
