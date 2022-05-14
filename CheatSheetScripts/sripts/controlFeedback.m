function posehist= controlFeedback(current_pos,target)

% loading the mat file with the position history and the robotpar
load('posehist.mat','posehist');
load('robotpar.mat','robotpar');

K_p = 3;
K_alpha =8;
K_beta = -1.5;


p = 10;
teta_T = 10;
i = 1;

while p > 0.01 || teta_T > 0.02

    disp("Iteration number: " + i);

    % loading the mat file with the position history
    load('posehist.mat','posehist')

    current_pos = posehist(end,:);

    Pos = [current_pos(1)-target(1);
        current_pos(2)-target(2)];

    T = [ cos(target(3)) sin(target(3));
        -sin(target(3)) cos(target(3))]*Pos;

    teta_T = current_pos(3)-target(3);

    p = sqrt(T(1)^2+T(2)^2);
    alpha = -teta_T + atan2(-T(2),-T(1));
    beta = -teta_T - alpha;

    %disp("p =" + p + " alpha: = " + alpha + "beta= " + beta);

    v = K_p*p;
    if v < 0.005
        v = 0;
        break
    end

    w = K_alpha*alpha + K_beta*beta;
    disp("omega = " + w)

    % loading the mat file with the position history
    load('posehist.mat','posehist')
    load('robotpar.mat','robotpar')

    ts = 0.01;

    %Setting the wheelspeed as v +/- w
    wr = (v + w*(robotpar(1)/2))/robotpar(2);
    wl = (v - w*(robotpar(1)/2))/robotpar(3);
    wheelspeed = [wr wl];
    %Last row of the posehist array, we should start adding from the next
    %one:
    starting_pos = size(posehist,1);

    %Evaluating the new position:
    posehist(starting_pos+1,:) = kinupdate(posehist(starting_pos,:),robotpar,ts,wheelspeed);

    save('posehist.mat','posehist');

    %Updating the loop condition:
    current_pos = posehist(end,:);
    Pos = [current_pos(1)-target(1);
        current_pos(2)-target(2)];
    T = [ cos(target(3)) sin(target(3));
        -sin(target(3)) cos(target(3))]*Pos;

    teta_T = current_pos(3)-target(3);
    p = sqrt(T(1)^2+T(2)^2);

    %disp("The loop condition are: Teta_T =" + teta_T + " p =" + p)

    i = i +1;
end


end