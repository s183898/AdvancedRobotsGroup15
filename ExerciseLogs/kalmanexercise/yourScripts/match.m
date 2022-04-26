function [matchResult] = match(pose,poseCov,worldLines,laserLines)
% [matchResult] = MATCH(pose,poseCov,worldLines,laserLines)
%   This function matches the predicted lines to the extracted lines. The
%   criterion for a match is the mahalanobis distance between the (alpha,
%   r) parameters of the predicted and the extracted lines. The arguments
%   are:
%       pose: The estimated robot pose given as [x,y,theta]
%       poseCov: The estimated covariance matrix of the robot pose
%       worldLines: Known world lines in world coordinates, given as
%       [alpha;r] for each line. Number of rows = number of lines
%       laserLines: Lines extracted from the laser scan. Given as [alpha;r]
%       for each line. Number of rows = number of lines
%
%       matchResult: A (5xnoOfWorldLines) matrix whose columns are 
%       individual pairs of line matches. It is structured as follows:
%       matchResult = [ worldLine(1,1) , worldLine(1,2) ...  ]
%                     [ worldLine(2,1) , worldLine(2,2)      ]
%                     [ innovation1(1) , innovation2(1)      ]
%                     [ innovation1(2) , innovation2(2)      ]
%                     [ matchIndex1    , matchIndex2    ...  ]
%           Note that the worldLines are in the world coordinates!

%     The varAlpha and varR are the assumed variances of the parameters of
%     the extracted lines, they are read globally.
%     sumR Is assumed to be a constant diagonal matrix with diagonal elements
%     varAlpha and varR defined in constants.m

    global varAlpha varR
    
    sigmaR = [varAlpha 0; 0 varR];
    matchResTmp = zeros(length(laserLines),5);
    g = 2;
    
    for p = 1:length(worldLines)
    [projectedLine, lineCov] = projectToLaser(worldLines(:,p),pose,poseCov);
    innoCov = lineCov + sigmaR;
        for q = 1:size(laserLines,2)
            innoAlpha = laserLines(1,q) - projectedLine(1);
            innoR = laserLines(2,q) - projectedLine(2);
            vijt = [innoAlpha innoR];
            MahaDist = vijt.*(innoCov^-1).*vijt';
            if MahaDist <= g^2
                matchResTmp(q,:)= [worldLines(:,p)' vijt q];
                break
            end
        end
    end
    
    matchResult = matchResTmp';
    
end
