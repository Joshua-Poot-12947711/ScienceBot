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
               plot(self.model,q, 'scale', 2, 'noarrow');
           end
           
           %% Get Dobot Parameters and create seriallink
           function getDobot(self)
               %create Dobot name after 1ms pause
               pause(0.001);
               name = ['Dobot_',datestr(now,'yyyy')];
               
               L1 = Link('d',0,'a',0.057,'alpha',0,'offset',0,'qlim',[deg2rad(-135),deg2rad(135)]);
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
        function result = IsCollision(self, robot, linkPoses, qMatrix, faces, vertex, faceNormals, returnOnceFound)
            if nargin < 6
                returnOnceFound = true;
            end
            result = false;
            
            for qIndex = 1:size(qMatrix,1)
                % Get the transform of every joint (i.e. start and end of every link)
                tr = GetLinkPoses(qMatrix(qIndex,:), robot)
                
                % Go through each link and also each triangle face
                for i = 1 : size(tr,3)-1
                    for faceIndex = 1:size(faces,1)
                        vertOnPlane = vertex(faces(faceIndex,1)',:);
                        [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)');
                        disp(check)
                        if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                            plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                            display('The Lightscreen has been Triggered');
                            result = true;
                            if returnOnceFound
                                return
                            end
                        end
                    end
                end
            end
        end
        
        %% IsIntersectionPointInsideTriangle
        % Checks to see if a point (i.e. robot link) is on the same plane
        % as the triangle AND determines if the point is inside or outside
        % the triangle bounds.
        function result = IsIntersectionPointInsideTriangle(self, intersectP,triangleVerts)
            
            u = triangleVerts(2,:) - triangleVerts(1,:);
            v = triangleVerts(3,:) - triangleVerts(1,:);
            
            uu = dot(u,u);
            uv = dot(u,v);
            vv = dot(v,v);
            
            w = intersectP - triangleVerts(1,:);
            wu = dot(w,u);
            wv = dot(w,v);
            
            D = uv * uv - uu * vv;
            
            % Get and test parametric coords (s and t)
            s = (uv * wv - vv * wu) / D;
            if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
                result = 0;
                return;
            end
            
            t = (uv * wu - uu * wv) / D;
            if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
                result = 0;
                return;
            end
            
            result = 1;                      % intersectP is in Triangle
        end
        
        %% GetLinkPoses
        % q - robot joint angles
        % robot -  seriallink robot model
        % transforms - list of transforms
        function [transforms] = GetLinkPoses(self, q, robot)
            
            links = robot.links;
            transforms = zeros(4, 4, length(links) + 1);
            transforms(:,:,1) = robot.base;
            
            for i = 1:length(links)
                L = links(1,i);
                
                current_transform = transforms(:,:, i);
                
                current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
                    transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
                transforms(:,:,i + 1) = current_transform;
            end
        end
        %% LinePlaneIntersection
        % Given a plane (normal and point) and two points that make up another line, get the intersection
        % Check == 0 if there is no intersection
        % Check == 1 if there is a line plane intersection between the two points
        % Check == 2 if the segment lies in the plane (always intersecting)
        % Check == 3 if there is intersection point which lies outside line segment
        function [intersectionPoint,check] = LinePlaneIntersection(self, planeNormal,pointOnPlane,point1OnLine,point2OnLine)
            
            intersectionPoint = [0 0 0];
            u = point2OnLine - point1OnLine;
            w = point1OnLine - pointOnPlane;
            D = dot(planeNormal,u);
            N = -dot(planeNormal,w);
            check = 0; %#ok<NASGU>
            if abs(D) < 10^-7        % The segment is parallel to plane
                if N == 0           % The segment lies in plane
                    check = 2;
                    return
                else
                    check = 0;       %no intersection
                    return
                end
            end
            
            %compute the intersection parameter
            sI = N / D;
            intersectionPoint = point1OnLine + sI.*u;
            
            if (sI < 0 || sI > 1)
                check= 3;          %The intersection point  lies outside the segment, so there is no intersection
            else
                check=1;
            end
        end


                   
           %% Collision Detection
           function hasCollided = checkForCollision(self, q)
               %%plot cube in workspace to check for collisions
               [Y,Z] = meshgrid(-0.75:0.05:0.75,-0.75:0.05:0.75);
               sizeMat = size(Y);
               X = repmat(0.75,sizeMat(1),sizeMat(2));
               oneSideOfCube_h = surf(X,Y,Z);

               % Combine one surface as a point cloud
               cubePoints = [X(:),Y(:),Z(:)];

               % Make a cube by rotating the single side by 0,90,180,270, and around y to make the top and bottom faces
               cubePoints = [ cubePoints ...
                           ; cubePoints * rotz(pi/2)...
                           ; cubePoints * rotz(pi) ...
                           ; cubePoints * rotz(3*pi/2) ...
                           ; cubePoints * roty(pi/2) ...
                           ; cubePoints * roty(-pi/2)];         
         
               % Plot the cube's point cloud         
               cubeAtOigin_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'r.');
               cubePoints = cubePoints + repmat([2,0,-0.5],size(cubePoints,1),1);
               cube_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'b.');
               axis equal
               
               robot = self.model;

               % New values for the ellipsoid (guessed these, need proper model to work out correctly)
               centerPoint = [0,0,0];
               radii = [1,0.5,0.5];
               [X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
               for i = 1:4
                  robot.points{i} = [X(:),Y(:),Z(:)];
                  warning off
                  robot.faces{i} = delaunay(robot.points{i});    
                  warning on;
               end

               %robot.plot3d(q);
               %axis equal
               %camlight

               tr = zeros(4,4,robot.n+1);
               tr(:,:,1) = robot.base;
               L = robot.links;
               for i = 1 : robot.n
                  tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
               end

               % Go through each ellipsoid
               for i = 1: size(tr,3)
                 cubePointsAndOnes = [inv(tr(:,:,i)) * [cubePoints,ones(size(cubePoints,1),1)]']';
                 updatedCubePoints = cubePointsAndOnes(:,1:3);
                 algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radii);
                 pointsInside = find(algebraicDist < 1);
                 %display(['2.10: There are ', num2str(size(pointsInside,1)),' points inside the ',num2str(i),'th ellipsoid']);
                 if pointsInside > 0
                     hasCollided = true;
                 elseif pointsInside == 0
                     hasCollided = false;
                 end
               end
           end
       end
end
