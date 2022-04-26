function odoTargetPose = trans(transform,targetPose)
% odoTargetPose = trans(transform,targetPose)
% Transform a given point in world coordinates (targetPose) to odometry
% coordinates, using the origo of the odometry coordinates in world
% coordinates (transform).

    xT = transform(1); 
    yT = transform(2); 
    tT = transform(3);
    xw = targetPose(1); 
    yw =targetPose(2); 
    tw = targetPose(3);
    
    xo = xT + xw*cos(tT)-yw*sin(tT);
    yo = yT + yw*cos(tT)+xw*sin(tT);
    to = wrapToPi(tw + tT);
    odoTargetPose = [xo;yo;to];
    
end