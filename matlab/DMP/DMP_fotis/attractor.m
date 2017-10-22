function f =  attractor(N,h,c,w,y0,y_g,x)
    
    if length(w)==1
       w = w*ones(1,N);  
    end
    
    for d=1:size(w,1) %eacj dmp

        for n=1:N
           Psi(n) = exp(-h(n)*(x-c(n))^2);
        end

        f(d) = dot(Psi,w(d,:))/sum(Psi)*x*(y_g(d)-y0(d)) ;
        
    end