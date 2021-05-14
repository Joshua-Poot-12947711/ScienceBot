%% Loop to get live ar tag poses 
clear all;
clc;

%get ar tag information from ar_track_alvar_msgs/AlvarMarkers

ARTagSub = rossubscriber('/tags','geometry_msgs/PoseArray');
plotHandle = trplot(eye(4));
desiredTagPose = eye(4);
while (true)
    tagMsg = receive(ARTagSub);
    numTags = size(tagMsg.Poses);
    while numTags(1) < 1
        pause(1);
        tagMsg = receive(ARTagSub);
        numTags = size(tagMsg.Poses);
    end
    currentPose = tagMsg.Poses(1);
    currentPosition = currentPose.Position
    currentOrientation = currentPose.Orientation;
    rotMatrix = quat2rotm([currentOrientation.X currentOrientation.Y currentOrientation.Z currentOrientation.W]);
    homMatrix = rotMatrix;
    homMatrix(1,4) = currentPosition.X;
    homMatrix(2,4) = currentPosition.Y;
    homMatrix(3,4) = currentPosition.Z;
    homMatrix(4,4) = 1
    %homMatrix is the pose of the tag from the camera in the camera frame
    %as a homogeneous transform
    
end