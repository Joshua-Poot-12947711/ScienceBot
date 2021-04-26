%Hand-To-Eye Calibration Script
%Assuming marker 0 is attached to dobot end-effector, and markers 1 onwards are used for the 
%for the objects being retrieved/places to put the object

%get joint information form dobot subscriber here
%dobotSub = rossubscriber()

%get ar tag information from ar_track_alvar_msgs/AlvarMarkers
ARTagSub = ('/ar_pose_marker','ar_track_alvar_msgs/AlvarMarkers');
msg = recieve(ARTagSub);

% Transform for Camera to AR Tag on EndEff

trans_cam_endeff = [msg.Markers.Pose.Pose.Position.X, msg.Markers.Pose.Pose.Position.Y,...
    msg.Markers.Pose.Pose.Position.Z]
rot_cam_endeff = quat2rotm(msg.Markers.Pose.Pose.Orientation.W, ...
    msg.Markers.Pose.Pose.Orientation.X, ...
    msg.Markers.Pose.Pose.Orientation.Y, ...
    msg.Markers.Pose.Pose.Orientation.Z)

tr_cam_endeff = rt2tr(rot_cam_endeff, trans_cam_endeff);

%Transform for Camera to 
