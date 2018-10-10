% This function returns the value of v0 for all the defined panels

function [v0, r_pj] = get_v0(panel_points, middle_points)

    for i=1:(length(panel_points)-1) %For all panels
        for j=1:length(middle_points) %For all middle points

        vectorPanel_x = panel_points(i+1, 1) - panel_points(i,1); %X Coordinate Vector Panel
        vectorPanel_y = panel_points(i+1, 2) - panel_points(i,2); %Y Coordinate Vector Panel
        vectorPanel = [vectorPanel_x, vectorPanel_y]; %Vector Panel
        
        
        vectorPoint_x = middle_points(j,1) - panel_points(i,1); %X Coordinate Vector Point
        vectorPoint_y = middle_points(j,2) - panel_points(i,2); %Y Coordinate Vector Point
        vectorPoint = [vectorPoint_x, vectorPoint_y]; %Vector Point
        
        u_vectorPanel = norm(vectorPanel); %We compute its modulus
        u_vectorPoint = norm(vectorPoint); %We compute its modulus

        dotProduct = dot(vectorPanel, vectorPoint); %Computing the dot product
        normVectorProduct = u_vectorPanel * u_vectorPoint; %We compute the denominator
        
        valor = (dotProduct/normVectorProduct);
        %fprintf('\nValor: %f', valor);
        angle = acos(dotProduct/normVectorProduct); %Finally computing the angle
              
        
        v0(i,j) = angle; %Adding the angle to the matrix V0
        r_pj(i,j) = norm(vectorPoint);
        
        end
    end   
end