function [ poseOut, covOut ] = positionPrediction( poseIn,covIn,delSr,delSl)
%[poseOut, covOut] = POSITIONPREDICTION(poseIn,covIn,delSr,delSl) perform
%one step of robot pose prediction from a set of wheel displacements
%   poseIn = old robot pose
%   covIn = uncertainty on the old robot pose
%   delSr = right wheel linear displacement
%   delSl = left wheel linear displacement


%% Constants
% The robot parameters are read globally, odoB is the wheel separation, kR
% and kL are the odometry uncertainty parameters
global odoB kR kL kR_kf kL_kf

%% pose update
%Eqn 5.89 pp: 353
b = odoB;
delx = ((delSr+delSl)/2)*cos(poseIn(3)+((delSr-delSl)/(2*b))); % delx, dely, delt = path traveled in the last sampling interval
dely = ((delSr+delSl)/2)*sin(poseIn(3)+((delSr-delSl)/(2*b)));  % b or odoB is the distance between the two wheels of differential-drive robot
delt = (delSr-delSl)/b;

poseOut = [poseIn(1)+delx; poseIn(2)+dely; wrapToPi(poseIn(3)+delt)];
        
%% Covariance update
% F_x1, F_y1 and F_z1 are jacobians of f

%Eqn 5.14 pp:290
dels = (delSr+delSl)/2; % Traveled distance for the wheels
delt = (delSr-delSl)/b;

%Eqn 5.10 pp:289
Fp = [1 0 -dels*sin(poseOut(3)+delt/2);
      0 1 dels*cos(poseOut(3)+delt/2);
      0 0 1];
  
%Eqn 5.11 pp:289
Fu = [(1/2)*cos(poseOut(3)+delt/2)-dels/(2*b)*sin(poseOut(3)+delt/2) (1/2)*cos(poseOut(3)+delt/2)+dels/(2*b)*sin(poseOut(3)+delt/2);
      (1/2)*sin(poseOut(3)+delt/2)+dels/(2*b)*cos(poseOut(3)+delt/2) (1/2)*sin(poseOut(3)+delt/2)-dels/(2*b)*cos(poseOut(3)+delt/2);
      1/b -1/b];
  
Qt = [kR_kf*abs(delSr) 0;
      0 kL_kf*abs(delSl)];

%Eqn 5.90 pp: 354
covOut = Fp*covIn*Fp'+Fu*Qt*Fu';

end