function plotGMM(Mu, Sigma, color, display_mode);
%
% This function plots a representation of the components (means and 
% covariance amtrices) of a Gaussian Mixture Model (GMM) or a
% Gaussian Mixture Regression (GMR).
%
% Author:	Sylvain Calinon, 2009
%			http://programming-by-demonstration.org
%
% Inputs -----------------------------------------------------------------
%   o Mu:           D x K array representing the centers of the K GMM components.
%   o Sigma:        D x D x K array representing the covariance matrices of the 
%                   K GMM components.
%   o color:        Color used for the representation
%   o display_mode: Display mode (1 is used for a GMM, 2 is used for a GMR
%                   with a 2D representation and 3 is used for a GMR with a
%                   1D representation).++++4 is for GMR with a 3D
%                   representation,,5 is for 3D GMM 

nbData = size(Mu,2);%取列数
lightcolor = color + [0.6,0.6,0.6];
lightcolor(find(lightcolor>1.0)) = 1.0;

if display_mode==1
  nbDrawingSeg = 40;
  t = linspace(-pi, pi, nbDrawingSeg)';
  for j=1:nbData
    stdev = sqrtm(3.0.*Sigma(:,:,j));%均方根
    X = [cos(t) sin(t)] * real(stdev) + repmat(Mu(:,j)',nbDrawingSeg,1);%real取实部；repmat堆叠矩阵
    patch(X(:,1), X(:,2), lightcolor, 'lineWidth', 2, 'EdgeColor', color);
    plot(Mu(1,:), Mu(2,:), 'x', 'lineWidth', 2, 'color', color);
  end
elseif display_mode==2
  nbDrawingSeg = 40;
  t = linspace(-pi, pi, nbDrawingSeg)';
  for j=1:nbData
    stdev = sqrtm(3.0.*Sigma(:,:,j));
    X = [cos(t) sin(t)] * real(stdev) + repmat(Mu(:,j)',nbDrawingSeg,1);
    patch(X(:,1), X(:,2), lightcolor, 'LineStyle', 'none');
  end
  plot(Mu(1,:), Mu(2,:), '-', 'lineWidth', 3, 'color', color);
elseif display_mode==3
  for j=1:nbData
    ymax(j) = Mu(2,j) + sqrtm(3.*Sigma(1,1,j));
    ymin(j) = Mu(2,j) - sqrtm(3.*Sigma(1,1,j));
  end
  patch([Mu(1,1:end) Mu(1,end:-1:1)], [ymax(1:end) ymin(end:-1:1)], lightcolor, 'LineStyle', 'none');
  plot(Mu(1,:), Mu(2,:), '-', 'lineWidth', 3, 'color', color); 
  
  %此部分用于生成阻抗时序
Data_imp=zeros(2,length(Mu));
for i=1:length(Mu)  
    Data_imp(1,i)=Mu(1,i);
    Data_imp(2,i)=Mu(2,i);  
end  
  plot(Data_imp(1,:), Data_imp(2,:), '-', 'lineWidth', 3, 'color', color); 
save GMR_imp.mat Data_imp
  
  elseif display_mode==4
%   for j=1:nbData
%       h = plot_gaussian_ellipsoid( Mu(:,j)', Sigma(:,:,j)',2.5);
%   end
    plot3(Mu(1,:), Mu(2,:), Mu(3,:), '-', 'lineWidth', 3, 'color', color); 
    view(-85,53); set(gca,'proj','perspective'); grid on;
  grid on;  axis tight;
  
  
  elseif display_mode==5%正态分布画椭球的网址https://blog.csdn.net/liuxqsmile/article/details/1893411
  for j=1:nbData
      h = plot_gaussian_ellipsoid( Mu(:,j)', Sigma(:,:,j)',2.5);
  end
  view(-85,53); set(gca,'proj','perspective'); grid on;
  grid on;  axis tight;
end




