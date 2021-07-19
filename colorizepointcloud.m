function colors = colorizepointcloud(vertices, labels)
    colors = zeros(size(vertices,1), 3);
    for i = 1:size(vertices,1)
        if labels(i,1) == 1 %%&& vertices(i,3) > 2 && vertices(i,3) < 3
            if vertices(i,1) < 2 && vertices(i,1) > -6.3  && vertices(i,3) > 2.3 && vertices(i,3) < 2.4
                if vertices(i,2) < -3
                    colors(i, :) = [0 0 1]; %%% mple: flat mprosta dromos
                else
                    colors(i, :) = [1 0 1]; %%% mple: flat mprosta dromos
                end
                
            elseif vertices(i,1) < 2 && vertices(i,1) > -6.3  && vertices(i,3) < 2.3
                colors(i, :) = [0 1 1];
            elseif vertices(i,1) < 2 && vertices(i,1) > -6.3  && vertices(i,3) >= 2.3 
                colors(i, :) = [1 1 0];
            else
                colors(i, :) = [0 1 0];
            end
        else
            if vertices(i,3) >= 2.3 %% this is the height of lidar
                colors(i, :) = [1 1 0];
            elseif  vertices(i,1) < 2 && vertices(i,1) > -6.3  && vertices(i,3) < 2.3 
                colors(i, :) = [0 1 1];
            else
                colors(i, :) = [1 0 0];
            end
        end
    end
end