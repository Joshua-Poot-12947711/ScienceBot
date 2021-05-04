<<<<<<< HEAD
%Hand-To-Eye Calibration Script
%Assuming marker 0 is attached to dobot end-effector, and markers 1 onwards are used for the 
%for the objects being retrieved/places to put the object

%% Checkerboard Calibration for Camera to End Effector
function [cameraParams, tr_cam_endeff, ARTagPoses] = HandToEyeCalibration()
imageFolder = './CalibrationImages';

% Square Size is the size of each square on the checkerboard in mm
squareSize = 20;
% converting square size to m
squareSize = squareSize/1000;

% get the calibration images
filePattern = fullfile(imageFolder, '*.png');
calibImages = dir(filePattern);
for i = length(calibImages)
    baseFileName = calibImages(k).name;
    fullFileName = fullfile(myFolder, baseFileName);
    imageArray = imread(fullFileName);
    imShow(imageArray);
    drawnow;
end

% find checkerboard points
[points, boardSize, imagesUsed] = detectCheckerboardPoints(calibImages);
if(imagesUsed == 0)
    error("No calibration images are found in the folder ./Calibration Images");
end

% generate an ideal checkerboard to compare calibration images with
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% find camera parameters
cameraParams = estimateCameraParameters(points, worldPoints, 'WorldUnits', m, 'NumRadialDistortionCoefficients', 2, ...
    'EstimateTangentialDistortion', true);
%% Hand to Eye Transformations

L1 = Link('d',0,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-90),deg2rad(90)]);
L2 = Link('d',0,'a',0.16828,'alpha',0,'offset',0,'qlim',[deg2rad(0),deg2rad(85)]);
L3 = Link('d',0,'a',0.190,'alpha',0,'offset',0,'qlim',[deg2rad(-10),deg2rad(95)]);
L4 = Link('d',0,'a',0.03343,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-90),deg2rad(90)]);
dobot = SerialLink([L1 L2 L3 L4], 'name', 'dobotMagician');

%get joint information form dobot subscriber here
dobotSub = rossubscriber('/dobot_magician/joint_states');
pause(2);
currentJointState = jointStateSubscriber.LatestMessage.Position;

%get ar tag information from ar_track_alvar_msgs/AlvarMarkers
ARTagSub = rossubscriber('/ar_pose_marker','ar_track_alvar_msgs/AlvarMarkers');
tagMsg = recieve(ARTagSub);

% Transform for Camera to AR Tag on EndEff - assuming 0 is the base AR Tag

%offset of the tag to the base in m
offset = [0.1, 0.1];

trans_cam_dobotBase = [tagMsg.markers[0].Pose.Pose.Position.X, tagMsg.markers[0].Pose.Pose.Position.Y,...
    tagMsg.markers[0].Pose.Pose.Position.Z]
rot_cam_dobotBase = quat2rotm(tagMsg.Markers[0].Pose.Pose.Orientation.W, ...
    tagMsg.markers[0].Pose.Pose.Orientation.X, ...
    tagMsg.markers[0].Pose.Pose.Orientation.Y, ...
    tagMsg.markers[0].Pose.Pose.Orientation.Z);

tr_cam_dobotBase = rt2tr(rot_cam_dobotBase, trans_cam_dobotBase);
tr_cam_dobotBase = tr_cam_dobotBase*transl(offset(1), offset(2), 0);

dobot.base = tr_cam_dobotBase;

tr_cam_endeff = dobot.fkine(currentJointState);
% Transform for Camera to Object - assuming 1 is object AR Tag
% cell array for ar tag transforms 
ARTagPoses = cell(size(tagMsg.markers,2),1);
for i = 1:1:size(tagMsg.markers,2)
    trans_cam_object = [tagMsg.markers[i].Pose.Pose.Position.X, tagMsg.markers[i].Pose.Pose.Position.Y,...
        tagMsg.markers[i].Pose.Pose.Position.Z]
    rot_cam_object = quat2rotm(tagMsg.markers[i].Pose.Pose.Orientation.W, ...
        tagMsg.markers[i].Pose.Pose.Orientation.X, ...
        tagMsg.markers[i].Pose.Pose.Orientation.Y, ...
        tagMsg.markers[i].Pose.Pose.Orientation.Z);
    ARTagPoses{i} = rt2tr(rot_cam_object, trans_cam_object);
end
=======
%Hand-To-Eye Calibration Script
%Assuming marker 0 is attached to dobot end-effector, and markers 1 onwards are used for the 
%for the objects being retrieved/places to put the object

%% Checkerboard Calibration for Camera to End Effector
function [cameraParams, tr_cam_endeff, ARTagPoses] = HandToEyeCalibration(dobotM)
clear all

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
%% Hand to Eye Transformations

%get joint information form dobot subscriber here
dobotSub = rossubscriber('/dobot_magician/joint_states');
pause(2);
currentJointState = jointStateSubscriber.LatestMessage.Position;

%get ar tag information from ar_track_alvar_msgs/AlvarMarkers
ARTagSub = rossubscriber('/ar_pose_marker','ar_track_alvar_msgs/AlvarMarkers');
tagMsg = recieve(ARTagSub);

% Transform for Camera to AR Tag at base - assuming 0 is the base AR Tag
tagArray = {tagMsg.markers}
%offset of the tag to the base in m
offset = [0.1, 0.1];

trans_cam_dobotBase = [tagArray{0}.Pose.Pose.Position.X, tagArray{0}.Pose.Pose.Position.Y,...
    tagArray{0}.Pose.Pose.Position.Z]
rot_cam_dobotBase = quat2rotm(tagMsg.Markers[0].Pose.Pose.Orientation.W, ...
    tagArray{0}.Pose.Pose.Orientation.X, ...
    tagArray{0}.Pose.Pose.Orientation.Y, ...
    tagArray{0}.Pose.Pose.Orientation.Z);

tr_cam_dobotBase = rt2tr(rot_cam_dobotBase, trans_cam_dobotBase);
tr_cam_dobotBase = tr_cam_dobotBase*transl(offset(1), offset(2), 0);

dobotM.model.base = tr_cam_dobotBase;

tr_cam_endeff = dobotM.model.fkine(currentJointState);
% Transform for Camera to Object - assuming 1 is object AR Tag
% cell array for ar tag transforms 
ARTagPoses = cell(size(tagMsg.markers,2),1);
for i = 1:1:size(tagMsg.markers,2)
    trans_cam_object = [tagArray{i}.Pose.Pose.Position.X, tagArray{i}.Pose.Pose.Position.Y,...
        tagArray{i}.Pose.Pose.Position.Z]
    rot_cam_object = quat2rotm(tagArray{i}.Pose.Pose.Orientation.W, ...
        tagArray{i}.Pose.Pose.Orientation.X, ...
        tagArray{i}.Pose.Pose.Orientation.Y, ...
        tagArray{i}.Pose.Pose.Orientation.Z);
    ARTagPoses{i} = rt2tr(rot_cam_object, trans_cam_object);
end
>>>>>>> 683dfe948f3013b39f53c09dcbfedffced50415e
end