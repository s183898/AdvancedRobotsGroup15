function posehist = forward(distance, speed)

% loading the mat file with the position history and the robotpar
load('posehist.mat','posehist');
load('robotpar.mat','robotpar');

ts = 0.01;

%setting the wheelspeed, this changes if the distance is negative
if distance < 0
    wheelspeed = [-speed/robotpar(2) -speed/robotpar(3)];
end
if distance > 0
    wheelspeed = [speed/robotpar(2) speed/robotpar(3)];
end

% Evaluating the remaining distance to drive:
driven_dist = 0;
distance_diff = abs(distance - driven_dist)

while distance_diff > 0
    %Last row of the posehist array, we should start adding from the next
    %one:
    starting_pos = size(posehist,1);

    %Evaluating the new position:
    posehist(starting_pos+1,:) = kinupdate(posehist(starting_pos,:),robotpar,ts,wheelspeed);

    %Updating the loop condition:
    driven_dist = speed*ts;

    %Evaluating the remaining distance to drive:
    distance_diff = distance_diff - driven_dist
end

save('posehist.mat','posehist');

end