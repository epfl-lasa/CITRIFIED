clear all;close all;clc;

% data

x=[2.5;0.5;2.2;1.9;3.1;2.3;2;1;1.5;1.1];
y=[2.4;0.7;2.9;2.2;3.0;2.7;1.6;1.1;1.6;0.9];

d=[x y];

figure(1)
plot(x',y','b*')
grid on
xlabel('x')
ylabel('y')
axis([-4,4,-4,4])
title('data')
legend('original data','Location','southeast')
hold on
disp('prsee a key to continue')
pause;
plot(x-mean(x),y-mean(y),'r*')
legend('original data','centrilized data','Location','southeast')
disp('prsee a key to continue')
% pause;
% plot((x-mean(x))/sqrt(cov_matrix(x)),(y-mean(y))/cov_matrix(y),'g*')
% legend('original data','centralized data','standardized data','Location','southeast')


[w,e]=myPCA(d,'');


create_arrow(1,[0 0],w(:,1),e(1),'g',2)
create_arrow(1,[0 0],w(:,2),e(2),'k',2)