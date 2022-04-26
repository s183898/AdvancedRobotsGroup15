function line = lsqLine(points)
    % New lsqLine function with using the matlab functions instead of
    % operators. This apparently works :) And you will not get errors from
    % ransacLines function in the 'extras' folder.
    
    x = points(1,:);
    y = points(2,:);

    n = length(points);
    
    xmean = mean(x);
    ymean = mean(y);
    
    sumx = sum(x);
    sumy = sum(y);
    
    sumx2 = x*x';
    sumy2 = y*y';
    
    sumxy = x*y';
    
    alpha = (1/2) * atan2((2*sumx*sumy-2*n*sumxy),(sumx^2-sumy^2-n*sumx2+n*sumy2));
    
    r = xmean*cos(alpha) + ymean*sin(alpha);
    
    if r < 0
        r = abs(r);
        if alpha<0
            alpha = alpha + pi;
        else
            alpha = alpha - pi;
        end
    end
    
    line = [alpha,r]; 
end