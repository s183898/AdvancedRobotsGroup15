function line = lsqline(points)

x = points(1,:);
y = points(2,:);

n = length(points);

sumx = sum(x);
sxsquared = x*x';

sumy = sum(y);
sysquared = y*y';
sumxy = x*y';

xh = mean(x);
yh = mean(y);

num = (2*sumx*sumy - 2*n*sumxy);
denum = sumx^2 - sumy^2 - n*sxsquared + n*sysquared;

alpha = (atan2(num,denum))/2;

r = xh*cos(alpha) + yh*sin(alpha);

if r < 0
    r = -r;

    if alpha < 0
        alpha = alpha + pi;
    else
        alpha = alpha - pi;
    end
end


line = [alpha, r];

end

