function points = get_ca_ij(r_values,th,min,max)
    points = [];
    n = 1;
    for i = 1:size(r_values,1);
        %disp(i + " : " + n)
        %disp(i)
        th_i = mod(th(i),360);
        if th_i == 0
            th_i = 360;
        end
        if sign(max)~=sign(min)
            if th_i < max && th_i>0
               points = [points i]; 
            elseif th_i>mod(min,360)
               points = [points i];   
            end
        else
            if th_i < mod(max,360) && th_i > mod(min,360)
               points = [points i]; 
            end
        end
        n = n+1;
    end
end