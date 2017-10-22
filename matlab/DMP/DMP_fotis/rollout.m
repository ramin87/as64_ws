%phase system (canonical system)
function [x, xd] = rollout(Ts, timesteps, type, ax)

if (nargin < 3 )
    ax = 1;
    type = 'exp';
end

x(1)=1; 
taf =1;

for i=2:timesteps
    if ( strcmp(type,'exp') ) %exponential
        xd(i) = -ax*x(i-1) / taf; 
        
    else %linear %if ( strcmp(type,'lin') )
        if (x(i-1)>0) 
            xd(i)=-1/(Ts*timesteps) /taf;
        else
            xd(i)=0;
        end
    end
    
    %integrate
    x(i)=x(i-1)+xd(i)*Ts;
end

x=x'; %column vector