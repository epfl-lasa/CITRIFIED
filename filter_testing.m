clear all;close all; clc
y = 3*ones(1,100) + randn(1,100);
figure
plot(1:100, y)
hold on; grid on;

freq = 9.380037948039710e+02;
[b, a] = butter(4, 6/freq*2);

y1 = filter(b, a, y);
% plot(1:100, y1)

y2 = flip(filter(b, a, flip(y1)));
% plot(1:100, y2)

plot(1:100, filtfilt(b,a,y))

% figure
% hold on;grid on;
% y_pad = [y(1)*ones(1,10) y y(end)*ones(1,10)];
% y1_pad = filter(b, a, y_pad);
% plot(1:120, y1_pad)
% 
% y2_pad = flip(filter(b, a, flip(y1_pad)));
% plot(1:120, y2_pad)
% 
% plot(1:120, filtfilt(b,a,y_pad))
ic1 = filtic(b,a, mean(y(1:4))*ones(4,1));
ic2 = filtic(b,a, mean(y(end-4:end))*ones(4,1));
y1_ic = filter(b,a,y,ic1);
y2_ic = flip(filter(b,a,flip(y1_ic),ic2));
% plot(1:100, filter(b,a,y, ic))
plot(1:100, y2_ic)