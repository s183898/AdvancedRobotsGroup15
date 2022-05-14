function posehist = turn(angle, speed)

% loading the mat file with the position history
load('posehist.mat','posehist')
load('robotpar.mat','robotpar')

ts = 0.01;

%setting the wheelspeed, this changes if the distance is negative
if angle < 0
    wheelspeed = [-speed/robotpar(2) speed/robotpar(3)];
end
if angle > 0
    wheelspeed = [speed/robotpar(2) -speed/robotpar(3)];
end

% Evaluating the remaining angle to turn:
target_angle = posehist(end,3) + angle;
angle_diff = angle;

if angle > 0
    while angle_diff > 0
        %Last row of the posehist array, we should start adding from the next
        %one:
        starting_pos = size(posehist,1);

        %Evaluating the new position:
        posehist(starting_pos+1,:) = kinupdate(posehist(starting_pos,:),robotpar,ts,wheelspeed);

        %Evaluating the remaining distance to drive:
        angle_diff = target_angle - posehist(end,3);
    end
end

if angle < 0
    while angle_diff < 0
        %Last row of the posehist array, we should start adding from the next
        %one:
        starting_pos = size(posehist,1);

        %Evaluating the new position:
        posehist(starting_pos+1,:) = kinupdate(posehist(starting_pos,:),robotpar,ts,wheelspeed);

        %Evaluating the remaining distance to drive:
        angle_diff = target_angle - posehist(end,3);
    end
end

save('posehist.mat','posehist');

end