%% Dobot Simulation Script
clc
clf
clear all

%% Load Environment, Robot and 'Light Screen'
simEnvironment = DobotEnvironment();
simEnvironment.loadEnvironmentObjects();
% lightScreen = simEnvironment.generateLightScreen([],[]);
%%
dobot = DobotSimulation();
dobot.model.base = dobot.model.base * transl(-0.75, 0, 1.05);
dobot.model.animate(dobot.model.getpos);

UR5 = LinearUR5(false);
UR5.model.base = UR5.model.base * transl(0, 1.02, 0.75) * troty(pi/2); %(y,z,x)
UR5.model.animate(UR5.model.getpos)
view(60, 30)
