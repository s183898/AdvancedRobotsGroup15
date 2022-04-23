function map = makeWave(map,start,goal,queue)
    queue = push(goal,queue);
    while ~isempty(queue)
        [cp,queue] = pop(queue);
        if cp == start
            map(goal(1),goal(2)) = 0;
            break
        end
       
        nbs = findNeighbours(map,cp(1),cp(2));
            for i = -1:1
                for j = -1:1
                    
                    x = cp(1)+i;
                    y = cp(2)+j;
                    
                    nbi = nbs(i+2,j+2);
                    
                    if ~(isnan(nbi))
                        d = map(cp(1),cp(2)) + sqrt(i^2+j^2);

                        if nbi == 0
                            map(x,y) = d;
                            queue = push(cp+[i,j],queue);
                        else
                            if (map(x,y)>d)
                                map(x,y)=d;
                            end
                        end        
                    end
                end
            end     
    end          
end  