clear

data = load('data_VFon.dat');

grip = data(:,15); %gripper status
dgrip = (diff(grip));

start = find(dgrip==1);
stop = find(dgrip==-1);

T = data(:,1);
P = data(:,2:4);
% F = data(start(1):end,5:7);
% P_ref  = data(start(2)+2:end,8:10);
% k = data(start(1):end,11); %k parameter "confidence" or "stiffness"
% Fnorm = data(start(1):end,12); %force_norm
% pe = data(start(1):end,13); %pe

segment_1 = P(start(1):stop(1),:);
segment_2 = P(stop(1):start(2),:);


data = segment_2';
data = data(:,1:3:end);
Ts = 0.001;
save('data.mat','data','Ts');

%%
figure(1); clf

subplot(3,1,1)
hold on
plot(P)
title('demonstrated position')
marks = [start];
plot(start,[0],'*r')
plot(stop,[0],'*g')
legend('X','Y','Z','cues')

subplot(3,1,2)
plot(segment_1)
title('Segment 1')

subplot(3,1,3)
plot(segment_2)
title('Segment 2')
