function transform = findTransform(odoPose, pose)
% transform = FINDTRANSFORM(odoPose,pose)
% Find the transformation from the world coordinates to the odometry
% coordinates given a pose in the odometry coordinates (odoPose) and the
% same point in the world coordinates (pose). The output (transform) is
% simply the origo of the odometry coordinates in the world coordinates
    
    xw = pose(1);
    yw = pose(2);
    tw = pose(3);
    xo = odoPose(1);
    yo = odoPose(2);
    to = odoPose(3);
    
    tT = to-tw;
    xT = xo-xw*cos(tT)+yw*sin(tT);
    yT = yo-yw*cos(tT)-xw*sin(tT);
    
    transform = [xT;yT;tT];
    
end