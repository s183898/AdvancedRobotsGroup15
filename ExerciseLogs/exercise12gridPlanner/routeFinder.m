function route = routeFinder(map,start,goal)
    route = [start];
    nextPoint = start;
   
        while ~(nextPoint(1) == goal(1) && nextPoint(2) == goal(2))
        
        nbs = findNeighbours(map,nextPoint(1),nextPoint(2));
            minD = Inf;
            for i = -1:1
                for j = -1:1
                    x = nextPoint(1)+i;
                    y = nextPoint(2)+j;
                    
                    nbi = nbs(i+2,j+2);
                    notGoal = ~(x == goal(1) && y == goal(2));
                    
                    if ~(isnan(nbi) || (nbi == 0 && notGoal))
                      if nbi < minD
                          minD = nbi;
                          xmin = x;
                          ymin = y;
                      end
                    end
                end
            end
            nextPoint = [xmin,ymin];
            route = push(nextPoint,route);
    end          
end     
