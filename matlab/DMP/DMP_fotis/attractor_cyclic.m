function f =  attractor_cyclic(N,h,c,w,x)
    
    if length(w)==1
       w = w*ones(1,N);  
    end
    
    for d=1:size(w,1) %eacj dmp

        for n=1:N
           Psi(n) = exp(h(n)*( cos(x-c(n)) -1 ) ); 
        end

        f(d) = dot(Psi,w(d,:))/sum(Psi);
        
    end