function [poseOut,poseCovOut] = measurementUpdate(poseIn,poseCovIn,matchResult)
%[ poseOut, poseCovOut ] =MEASUREMENTUPDATE ( poseIn, poseCovIn,
%matchResult ) perform extended Kalman filter measurement update on the
%estimated robot pose poseIn with covariance poseCovIn using a set of
%matched predicted and extracted laser scanner lines given in matchResult.
%The arguments are defined as:
%       poseIn: The estimated robot pose given as [x,y,theta]
%       poseCovIn: The estimated covariance matrix of the robot pose
%       matchResult: A (5xnoOfWorldLines) matrix whose columns are 
%       individual pairs of line matches. It is structured as follows:
%       matchResult = [ worldLine(1,1) , worldLine(1,2) ...  ]
%                     [ worldLine(2,1) , worldLine(2,2)      ]
%                     [ innovation1(1) , innovation2(1)      ]
%                     [ innovation1(2) , innovation2(2)      ]
%                     [ matchIndex1    , matchIndex2    ...  ]
%           Note that the worldLines are in the world coordinates!
%       
%       poseOut: The updated robot pose estimate
%       poseCovOut: The updated estimate of the robot pose covariance 
%       matrix 

    % Constants
    % The laser scanner pose in the robot frame is read globally(lsrRelpose)
    % The varAlpha and varR are the assumed variances of the parameters of
    % the extracted lines, they are also read globally
    
    % Constants:
    global lsrRelPose varAlpha varR
    
    SIZE = size(matchResult,2);
    nH = zeros(SIZE*2,3);
    sigmaIN = zeros(2*SIZE,2);
    sigmaR = zeros(SIZE*2,SIZE*2);
    
    for j = 1:SIZE
        sigmaR(j*2-1:j*2,j*2-1:j*2) = [varAlpha 0;0 varR];
    end
    
    % initial for loop
    for j = 1:SIZE
        [~, ~] = projectToLaser([matchResult(1,j) ; matchResult(2,j)],poseIn,poseCovIn)
        
        aw = matchResult(1,j);
        tw = poseIn(3);
        xr = lsrRelPose(1);
        yr = lsrRelPose(2);
        tr = lsrRelPose(3);
        
        nH(j*2-1:j*2,:) = [0 0 -1;
                          -cos(aw) -sin(aw) yr*cos(tr-aw+tw)+xr*sin(tr-aw+tw)];
    end
    
    %innovation covariance
    sigmaIN = nH*poseCovIn*(nH.')+sigmaR
    
    vt = [(matchResult(3,:)) (matchResult(4,:))]';
    
    % Eq. 5.86 page.336
    kT = poseCovIn*(nH.')*inv(sigmaIN)
    
    % Eq. 5.84 page.335
    xt = poseIn+kT*vt
    
    % Eq. 5.85 page.335
    Pt = poseCovIn-kT*sigmaIN*(kT.')
 
    poseOut = xt;
    poseCovOut = Pt;
end