% Test Script for image calibration
clear all

% setting the folder the images are in
imageFolder = './testCalibrationImages';

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
    error("No calibration images are found in the folder ./Calibration Images");
end

% generate an ideal checkerboard to compare calibration images with
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% find camera parameters
cameraParams = estimateCameraParameters(points, worldPoints, 'WorldUnits', 'm', 'NumRadialDistortionCoefficients', 2, ...
    'EstimateTangentialDistortion', true);