classdef DobotSimulation < handle

       properties(Access = private)
        %> Robot model
        model;
        
        %> Workspace
        workspace = [-2 2 -2 2 -2 2];   
        
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
           function getDobot()
               L1 = Link('d',0,'a',0.057,'alpha',0,'offset',0,'qlim',[deg2rad(-135),deg2rad(135)]);
               L2 = Link('d',0,'a',0,'alpha',0,'offset',-pi/2,'qlim',[deg2rad(5),deg2rad(80)]);
               L3 = Link('d',0,'a',0,'alpha',0,'offset',0,'qlim',[deg2rad(15),deg2rad(170)]);
               L4 = Link('d',0,'a',0,'alpha',0,'offset',-pi/2,'qlim',[pi/2,pi/2]);
               L5 = Link('d',0,'a',0,'alpha',0,'offset',-pi/2,'qlim',[deg2rad(--85),deg2rad(85)]);
               
               self.model = SerialLink([L1 L2 L3 L4 L5], 'name', name);
           end
           
           %% Dobot Fkine
           function translation = DobotFkine(q)
               
               x = self.armLengths(2)*cos(q(1))*sin(q(2))+self.armLengths(3)*cos(q(1))*cos(q(3));
               y = self.armLengths(2)*sin(q(1))*sin(q(2))+self.armLengths(3)*sin(q(1))*sin(q(3));
               z = self.armLengths(2)*cos(q(2)) - self.armLengths(3)*sin(q(3))-self.armLengths(1);
               
               translation = [x, y, z];
           end
           %% Dobot Ikine
           function jointAngles = DobotIkine(translation)
               %let translation = [x, y, z]
               [x, y, z] = [translation(1), translation(2), translation(3)];
               length = sqrt(x^2+y^2);
               distance = sqrt(length^2+z^2);
               t1 = atan(z/l);
               t2 = acosd(((armLength(1)^2)+distance^2-(armLength(2))^2)/(2*armLength(1)*distance));
               alpha = t1 + t2;
               beta = acosd(((armLength(1)^2)+(armLength(2)^2)-distance^2)/(2*armLength(1)*armLength(2)));
               jointAngles = [atan(y/x), (pi/2 - alpha), (pi - beta - alpha)];
           end
       end
end
