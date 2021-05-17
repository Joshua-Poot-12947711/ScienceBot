classdef DobotMagicianSerial < handle
    properties
        %> Robot model
        model;
        
        %> Workspace
        workspace = [-2 2 -2 2 -2 2];
        
        %> Link Lengths, change last length if a end effector is added
        armLengths = [0.08, 0.135, 0.147, 0, 0];
        
        %> Flag to indicate if gripper is used
        useGripper = false;
        
        r_qlim(1, :) = [0, 1];
        r_qlim(2, :) = deg2rad([-135 135]);
        r_qlim(3, :) = deg2rad([5 80]);
        r_qlim(4, :) = deg2rad([-5 85]);
        r_qlim(5, :) = deg2rad([-85 85])

    end
    
    methods%% Class for DoBot Magician robot simulation
        function self = dobotMagician(useGripper)
            if nargin < 1
                useGripper = false;
            end
            self.useGripper = useGripper;
            
            %> Define the boundaries of the workspace
            
            
            % robot =
            self.GetDoBotRobot();
            % robot =
            q = zeros(1,4);
            plot(self.model,q);
        end
        
        %% GetDoBotRobot
        % Given a name (optional), create and return a UR3 robot model
        function GetDoBotRobot(self)
            %     if nargin < 1
            % Create a unique name (ms timestamp after 1ms pause)
            pause(0.001);
            name = ['DoBot_',datestr(now,'yyyymmddTHHMMSSFFF')];
            %     end
            
            L1 = Link('d',0,'a',0.08,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-90),deg2rad(90)]);
            L2 = Link('d',0,'a',0.135,'alpha',0,'offset',0,'qlim',[deg2rad(0),deg2rad(85)]);
            L3 = Link('d',0,'a',0.147,'alpha',0,'offset',0,'qlim',[deg2rad(-10),deg2rad(95)]);
            L4 = Link('d',0,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-90),deg2rad(90)]);
            self.model = SerialLink([L1 L2 L3 L4], 'name', 'dobotMagician');
        end
        %% Dobot Manual Fkine
        function translation = DobotFkine(q)
            
            x = self.armLengths(2)*cos(q(1))*sin(q(2)) + self.armLengths(3)*cos(q(1))*cos(q(3));
            y = self.armLengths(2)*sin(q(1))*sin(q(2)) + self.armLengths(3)*sin(q(1))*sin(q(3));
            z = self.armLengths(2)*cos(q(2)) - self.armLengths(3)*sin(q(3)) - self.armLengths(1);
            
            translation = [x, y, z];
        end
        
        %% Dobot Manual Ikine
        function jointAngles = DobotIkine(translation)
            %let translation = [x, y, z]
            [x, y, z] = [translation(1), translation(2), translation(3)];
            length = sqrt(x^2+y^2);
            distance = sqrt(length^2+z^2);
            t1 = atan(z/l);
            t2 = acosd(((armLength(1)^2)+distance^2-(armLength(2))^2)/(2*armLength(1)*distance));
            alpha = t1 + t2;
            beta = acosd(((armLength(1)^2)+(armLength(2)^2)-distance^2)/(2*armLength(1)*armLength(2)));
            jointAngles = [atan(y/x), (pi/2 - alpha), (pi - beta - alpha)]
        end
        
        %% Dobot Ikcon
        function [qstar, error, exitflag] = ikcon(robot, T, q0)
            
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
    end
end
