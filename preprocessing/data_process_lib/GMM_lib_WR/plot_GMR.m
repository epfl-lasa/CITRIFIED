%% 画生成的轨迹和阻抗和最后的合成（这个是用高斯学习后得到的数据）
load('data/GMR_imp.mat'); 
Mu1=Data_imp;
load('data/GMR_trj.mat'); 
Mu2=Data_trj;
load('colormp');

subplot(3,1,1)
    plot3(Mu2(1,:), Mu2(2,:), Mu2(3,:), '-', 'lineWidth', 3); 
    view(-85,53); set(gca,'proj','perspective'); grid on;
  grid on;  axis tight;
subplot(3,1,2)
  plot(Mu1(1,:), Mu1(2,:), '-', 'lineWidth', 3); 
   
subplot(3,1,3)
    scatter3(Mu2(1,:), Mu2(2,:), Mu2(3,:),10,Mu1(2,:))
    colormap(Colormp/max(max(Colormp)))
    colorbar('location','EastOutside'); %Sometimes this messes with the plot. Try at your own risk
    caxis([0,max(Mu1(2,:))])
        xlabel('x(m) ');
ylabel('y(m)');
zlabel('z(m)');
title('Arm stiffness estimate')
view(-85,53)
set(gca, 'Fontname', 'Times newman', 'Fontsize', 20);
set(gca,'proj','perspective'); grid on;
grid on;  axis tight;

%% 将轨迹和阻抗编程一个可以直接使用的命令程序

  

