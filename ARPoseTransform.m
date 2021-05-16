%% test for ar tag pose transform
clear all;
clc;

rosinit;


%% ar tag transform
clc;
clear;

ARTagSub = rossubscriber('/tags','geometry_msgs/PoseArray');
tagMsg = receive(ARTagSub);

%tag5 = tagMsg.Poses(1);
%tag15 = tagMsg.Poses(11);

tag5Pose = tagMsg.Poses(1);
tag5Position = tag5Pose.Position;
tag5Orientation = tag5Pose.Orientation;
tag5RotMatrix = quat2rotm([tag5Orientation.X tag5Orientation.Y tag5Orientation.Z tag5Orientation.W]);
tag5HomMatrix = tag5RotMatrix;
tag5HomMatrix(1,4) = tag5Position.X;
tag5HomMatrix(2,4) = tag5Position.Y;
tag5HomMatrix(3,4) = tag5Position.Z;
tag5HomMatrix(4,4) = 1

tag15Pose = tagMsg.Poses(2);
tag15Position = tag15Pose.Position;
tag15Orientation = tag15Pose.Orientation;
tag15RotMatrix = quat2rotm([tag15Orientation.X tag15Orientation.Y tag15Orientation.Z tag15Orientation.W]);
tag15HomMatrix = tag15RotMatrix;
tag15HomMatrix(1,4) = tag15Position.X;
tag15HomMatrix(2,4) = tag15Position.Y;
tag15HomMatrix(3,4) = tag15Position.Z;
tag15HomMatrix(4,4) = 1

%relativeTr = tag5HomMatrix\tag15HomMatrix %same as inv(5)*15

relativeTr = inv(tag5HomMatrix)*tag15HomMatrix

