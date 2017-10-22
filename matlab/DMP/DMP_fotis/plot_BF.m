function plot_BF(h,c,Ts,tf,cs_type,ax,dmp_type)

% if nargin<3 %put some values for testing
%     Ts=0.01;
%     tf=5;
% end

if nargin<7
    dmp_type='discrete';
end

time = Ts:Ts:tf;
if (strcmp(dmp_type,'discrete'))
    [x_track, ~] = rollout(Ts, tf/Ts,cs_type,ax);
elseif (strcmp(dmp_type,'cyclic'))
    x_track = time;
else
    disp(['Unknown dmp_type:' dmp_type])
    return
end
   


figure(99);clf
for i=1:length(h)
    
   if (strcmp(dmp_type,'discrete'))
        psi(:,i) = exp(-h(i)*(x_track-c(i)).^2) ;
   else 
        psi(:,i) = exp(h(i)*( cos(x_track-c(i)) -1 ) ); 
   end
   
   subplot(2,1,1)
   hold on
   plot(time,psi(:,i))
   xlabel('t')
   
   subplot(2,1,2)
   hold on
   plot(x_track,psi(:,i))
   set(gca,'XDir','reverse')
   xlabel('x')
   
%    drawnow
%    pause(0.2)
end

