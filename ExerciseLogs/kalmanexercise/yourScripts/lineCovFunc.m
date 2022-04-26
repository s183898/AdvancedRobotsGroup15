function [sigmazp] = lineCovFunc(zw,pose,poseCov)
%LINECOV Summary of this function goes here
%   Detailed explanation goes here

global lsrRelPose % The laser scanner pose in the robot frame is read globally

aw = zw(1);
tw = pose(3);
xr = lsrRelPose(1);
yr = lsrRelPose(2);
tr = lsrRelPose(3);

nh = [0 0 -1;
    -cos(aw) -sin(aw) yr*cos(tr-aw+tw)+xr*sin(tr-aw+tw)];

sigmazp = nh*poseCov*nh';
end