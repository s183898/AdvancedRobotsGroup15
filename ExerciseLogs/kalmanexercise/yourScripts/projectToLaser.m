function [projectedLine,lineCov] = projectToLaser(worldLine,poseIn,covIn)
%[projectedLine, lineCov] = PROJECTTOLASER(worldLine,poseIn,covIn) 
%Project a word line to the laser scanner frame given the
%world line, the robot pose and robot pose covariance. Note that the laser
%scanner pose in the robot frame is read globally
%   worldLine: The line in world coordinates
%   poseIn: The robot pose
%   covIn: The robot pose covariance
%
%   projectedLine: The line parameters in the laser scanner frame
%   lineCov: The covariance of the line parameters

%% Constants
global lsrRelPose % The laser scanner pose in the robot frame is read globally

%% Line parameters in laser scanner frame

aw = worldLine(1);
rw = worldLine(2);
xw = poseIn(1);
yw = poseIn(2);
tw = poseIn(3);
xr = lsrRelPose(1);
yr = lsrRelPose(2);
tr = lsrRelPose(3);

ar = aw-tw;
al = ar-tr;
    
rr = rw - (xw*cos(aw) + yw*sin(aw));
% rl = rr - (xr*cos(alpha_r) + yr*sin(alpha_r));
rl = rr - (xr*cos(al) + yr*sin(al));

projectedLine = [al rl];
% lineCov = zeros(2,2);

%% Covariance of line parameters
zw = worldLine;
pose = poseIn;
poseCov = covIn;

sigmazp = lineCovFunc(zw,pose,poseCov);
lineCov = sigmazp;
end