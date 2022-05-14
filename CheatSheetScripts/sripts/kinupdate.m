
function newpose = kinupdate(pose,robotpar,ts,wheelspeed)

%% page 74
    J1 = [0 1 0;
          1 0 -robotpar(1)/2; %left wheel
          1 0 robotpar(1)/2]; %right wheel
    R = [cos(pose(3)) sin(pose(3)) 0;
         -sin(pose(3)) cos(pose(3)) 0;
         0 0 1];
    J2 = [ 0 robotpar(3)*wheelspeed(2) robotpar(2)*wheelspeed(1)]';
    newpose = pose' +  (inv(R)*inv(J1)*J2)*ts;
end