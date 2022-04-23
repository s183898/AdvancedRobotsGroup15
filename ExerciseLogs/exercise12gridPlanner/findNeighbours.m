function nb = findNeighbours(map,x,y)
    nb = zeros(3,3);
    
    for i = -1:1
        for j =-1:1
%             if j == 0 && i == 0
%                 continue
%             end
            try
                if (isnan(map(x+i,y+j)))
                    nb(i+2,j+2) = NaN;
                end
                nb(i+2,j+2) = map(x+i,y+j);
            catch
                nb(i+2,j+2) = NaN;
            end
        end
    end
end

