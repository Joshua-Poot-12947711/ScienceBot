%% Dobot Simulation Script
clc
clf
clear all

%% Load Environment, Robot and 'Light Screen'
simEnvironment = DobotEnvironment();
simEnvironment.loadEnvironmentObjects();
[vertex,faces,faceNormals] = simEnvironment.generateLightScreen([0.21, -0.8, 2.5],[0.2, 0.8, 1.1]);

dobot = DobotSimulation();
dobot.model.base = dobot.model.base * transl(-0.75, 0, 1.05);
dobot.model.animate(dobot.model.getpos);

UR5 = LinearUR5(false);
UR5.model.base = UR5.model.base * transl(0, 1.02, 0.75) * troty(pi/2); %(y,z,x)
UR5.model.animate(UR5.model.getpos)
view(60, 30)

%% Light Screen Detection
% Get the transform for each individual joint
transform = zeros(4,4,UR5.model.n+1);
transform(:,:,1) = UR5.model.base;
L = UR5.model.links;
q = UR5.model.getpos;
for i = 1 : UR5.model.n
        % The below function follows the dh parameter transformation, multiplied by the previous link transform
        %i.e. Trz * Tz * Tx * Trx
    transform(:,:,i+1) = transform(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
end

% generate goal transform, and joint angles
goalTr = transl(0, 0.3, 1.5);
q1 = UR5.model.getpos;
q2 = UR5.model.ikine(goalTr);

% increase steps until 
steps = 2;
while ~isempty(find(1 < abs(diff(rad2deg(jtraj(q1,q2,steps)))),1))
    steps = steps + 1;
end
qMatrix = jtraj(q1,q2,steps);

% Check if each link is within the triangle made by the three of the verticies of the rectangular prism (IsCollision)
% If there is a collision, stop the script to simulate the stopping of the robot
result = true(steps,1);
for i = 1: steps
    result(i) = dobot.IsCollision(UR5, transform, qMatrix(i,:),faces,vertex,faceNormals,false);
    UR5.model.animate(qMatrix(i,:));
    drawnow()
    if result(i) == true
        return
    end
end
