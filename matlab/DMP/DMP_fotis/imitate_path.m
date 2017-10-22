function f_d =  imitate_path(p,Ts,ay,by)

    if (size(p,1) < size(p,2))
        p = p'; %reshape to columns
    end
    
    y0 = p(1,:);
    y_g = p(end,:);
    
    pd = [zeros(1, size(p,2)); diff(p)/Ts];
    pdd = [zeros(1, size(p,2)); diff(pd)/Ts];
    
    f_d = pdd - ay*(by * (y_g - p) - pd);

