function [Mu,Sigma,Priors,expData,expSigma,data_mean,data_range] = gmm_gmr_F2mV(demos,input_dim,output_dim,verfine_data,nbStates,normalize,ellpsiod_magnitude_GMM,ellpsiod_magnitude_GMM_3D,ellpsiod_magnitude_GMR,ellpsiod_magnitude_GMR_3D)
%
% Demo of Gaussian Mixture Regression (GMR). 
% This source code is the implementation of the algorithms described in 
% Section 2.4, p.38 of the book "Robot Programming by Demonstration: A 
% Probabilistic Approach". 
%
% Author:	Sylvain Calinon, 2009
%			http://programming-by-demonstration.org
%
% The program loads a 3D dataset, trains a Gaussian Mixture Model 
% (GMM), and retrieves a generalized version of the dataset with associated 
% constraints through Gaussian Mixture Regression (GMR). Each datapoint 
% has 3 dimensions, consisting of 1 temporal value and 2 spatial values 
% (e.g. drawing on a 2D Cartesian plane). A sequence of temporal values is 
% used as query points to retrieve a sequence of expected spatial 
% distributiuon through Gaussian Mixture Regression (GMR).
%
% This source code is given for free! However, I would be grateful if you refer 
% to the book (or corresponding article) in any academic publication that uses 
% this code or part of it. Here are the corresponding BibTex references: 
%
% @book{Calinon09book,
%   author="S. Calinon",
%   title="Robot Programming by Demonstration: A Probabilistic Approach",
%   publisher="EPFL/CRC Press",
%   year="2009",
%   note="EPFL Press ISBN 978-2-940222-31-5, CRC Press ISBN 978-1-4398-0867-2"
% }
%
% @article{Calinon07,
%   title="On Learning, Representing and Generalizing a Task in a Humanoid Robot",
%   author="S. Calinon and F. Guenter and A. Billard",
%   journal="IEEE Transactions on Systems, Man and Cybernetics, Part B",
%   year="2007",
%   volume="37",
%   number="2",
%   pages="286--298",
% }

%% Definition of the number of components used in GMM.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% nbStates = 3;  %number of GMM (K)

%% Load a dataset consisting of 3 demonstrations of a 2D signal.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% load('data/data61_900.mat'); 
Data=demos(2:end,:);
nbVar = size(Data,1);

if mod(nbVar,3)==0
    plot_nbVar=nbVar/3;
else if nbVar==3+2
    plot_nbVar=2;
    else if nbVar==3+1
            plot_nbVar=2;
        end
    end
end

if mod(output_dim,3)==0
    plot_output=(input_dim+output_dim)/3;
else if output_dim==2 
    plot_output=2;
    else if output_dim==1
            plot_output=2;
        end
    end
end



%% normalize the data for better learning
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if normalize==1
    for input=1:input_dim+output_dim
        data_range(input) = max(Data(input,:)) - min(Data(input,:));
        data_mean(input) = mean(Data(input,:));
        m01 = (Data(input,:) - data_mean(input)) / data_range(input);
        Data(input,:)=m01;
        clear m01
    end
    for input=1:input_dim
        m01 = (verfine_data(input+1,:) - data_mean(input)) / data_range(input);
        verfine_data(input+1,:)=m01;
        clear m01
    end
    
else
    for input=1:input_dim+output_dim
        data_range(input)=1;data_mean(input)=0;
    end
    
end



%% Training of GMM by EM algorithm, initialized by k-means clustering.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[Priors, Mu, Sigma] = GMM_EM_init_kmeans(Data, nbStates);
[Priors, Mu, Sigma] = GMM_EM(Data, Priors, Mu, Sigma);

% number_windows=ceil(length(Data)/nbStates);
% for num_ellispiod_imp=1:nbStates 
%     Mu_time_plot(:,num_ellispiod_imp)=[demos(1,number_windows*(num_ellispiod_imp-1)+1)];
% end

%% Use of GMR to retrieve a generalized version of the data and associated
%% constraints. A sequence of temporal values is used as input, and the 
%% expected distribution is retrieved. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
expData(1:input_dim,:) = verfine_data(2:4,:);
[expData(input_dim+1:nbVar,:), expSigma] = GMM_GMR(Priors, Mu, Sigma,  expData(1:input_dim,:), [1:input_dim], [input_dim+1:nbVar]);


%% Plot of the data 1D
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
screen=get(0,'ScreenSize');
figure('position',[screen(:,1:2)+100 screen(:,3:4)/1.3],'name','data_GMM-time');
%plot 1D
for n=1:nbVar
  subplot(plot_nbVar,3,n); hold on;
  plot(demos(1,:), Data(n,:)*data_range(n)+data_mean(n), '*', 'markerSize', 1);%, 'color', [.3 .3 .3],'lineWidth', 3
  axis([min(demos(1,:)) max(demos(1,:)) min(Data(n,:)*data_range(n)+data_mean(n))-0.01 max(Data(n,:)*data_range(n)+data_mean(n))+0.01]);
  xlabel('p','fontsize',16); ylabel(['x_' num2str(n)],'fontsize',16);
end
% plot 3D
  subplot(plot_nbVar,3,n+1); hold on;grid on;
  plot3(Data(1,:),Data(2,:),Data(3,:), '*', 'markerSize', 1);%, 'color', [.3 .3 .3],'lineWidth', 3
%   axis([min(demos(1,:)) max(demos(1,:)) min(Data(n,:)*data_range(n)+data_mean(n))-0.01 max(Data(n,:)*data_range(n)+data_mean(n))+0.01]);
%   xlabel('p','fontsize',16); ylabel(['x_' num2str(n)],'fontsize',16);
view(9,57);


%% plot GMM and GMR
screen=get(0,'ScreenSize');
figure('position',[screen(:,1:2)+100 screen(:,3:4)/1.3],'name','data_GMM-force');
%plot 1D
for n=1:input_dim
  subplot(3,3,n); hold on; grid on;
  plot(Data(n,:), Data(input_dim+1,:), '*', 'markerSize', 1);%, 'color', [.3 .3 .3],'lineWidth', 3
 
 
end

for n=1:input_dim
  subplot(3,3,n); hold on;
  plotGMM_3D_P_add_IMP_adj_size([Mu(n,:);Mu(input_dim+1,:)], Sigma([n,input_dim+1],[n,input_dim+1],:), [0 .8 0],...
      1,0.1,ellpsiod_magnitude_GMM_3D);
  axis([min(Data(n,:)) max(Data(n,:)) min(Data(input_dim+1,:))-0.002 max(Data(input_dim+1,:))+0.002]);
  xlabel('p','fontsize',16); ylabel(['x_' num2str(n)],'fontsize',16);
  if n==1
        xlabel('F_X','fontsize',16); ylabel(['V'],'fontsize',16);grid on;
    elseif n==2
        xlabel('F_Y','fontsize',16); ylabel(['V'],'fontsize',16);grid on;
    elseif n==3
        xlabel('F_Z','fontsize',16); ylabel(['V'],'fontsize',16);grid on;
  end
    grid on;
end
% axis equal;

%plot 3D
% subplot(3,3,8); hold on;grid on;
%   plot3(Data(1,:),Data(2,:),Data(3,:), '*', 'markerSize', 1);%, 'color', [.3 .3 .3],'lineWidth', 3
% %   axis([min(demos(1,:)) max(demos(1,:)) min(Data(n,:)*data_range(n)+data_mean(n))-0.01 max(Data(n,:)*data_range(n)+data_mean(n))+0.01]);
% %   xlabel('p','fontsize',16); ylabel(['x_' num2str(n)],'fontsize',16);
% view(9,57);
% plotGMM_3D_P_add_IMP_adj_size(Mu(1:3,:), Sigma([1:3],[1:3],:), [0 .8 0],...
%       5,0.1,ellpsiod_magnitude_GMM_3D);


%% Plot of the GMM encoding results
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %plot 1D 
% for n=1:nbVar
%   subplot(plot_nbVar,3,n); hold on;
%   plotGMM_3D_P_add_IMP_adj_size([Mu(1,:);Mu(n,:)], Sigma([1,n],[1,n],:), [0 .8 0], 1,ellpsiod_magnitude_GMM,ellpsiod_magnitude_GMM_3D);
%   axis([min(Data(1,:)) max(Data(1,:)) min(Data(n,:))-0.01 max(Data(n,:))+0.01]);
%   xlabel('p','fontsize',16); ylabel(['x_' num2str(n)],'fontsize',16);
% end

%% Plot of the GMR regression results
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%plot 1D
% screen=get(0,'ScreenSize');
% figure('position',[screen(:,1:2)+100 screen(:,3:4)/1.3],'name','GMR-time');
for n=1:input_dim+output_dim
    if n<=input_dim
        subplot(3,3,n+3); hold on;
        plot(verfine_data(1,:), expData(n,:));
        axis([min(verfine_data(1,:)) max(verfine_data(1,:)) min(expData(n,:))-0.002 max(expData(n,:))+0.002]);
    else
      subplot(3,3,n+3); hold on;
      plotGMM_3D_P_add_IMP_adj_size([verfine_data(1,:); expData(n,:)], 0.3*expSigma(n-input_dim,n-input_dim,:), [0 0 .8], 3,ellpsiod_magnitude_GMR,ellpsiod_magnitude_GMR_3D);
      axis([min(verfine_data(1,:)) max(verfine_data(1,:)) min(expData(n,:))-0.002 max(expData(n,:))+0.002]);

      % plot ground turth
      hold on;
      plot(verfine_data(1,:), verfine_data(5,:),'r','linewidth',2);grid on;
    end  
    if n==1
        xlabel('t','fontsize',16); ylabel(['F_X'],'fontsize',16);grid on;
    elseif n==2
        xlabel('t','fontsize',16); ylabel(['F_Y'],'fontsize',16);grid on;
    elseif n==3
        xlabel('t','fontsize',16); ylabel(['F_Z'],'fontsize',16);grid on;
    end
end

screen=get(0,'ScreenSize');
figure
set(gcf,'unit','centimeters','position',[3 5 8*2 5*2])
 set(gca,'Position',[.15 .15 .8 .75],'FontName','Times New Roman','FontSize',12);
    
for n=1:input_dim+output_dim
    if n<=input_dim
        subplot(3,2,(n*2-1)); hold on;
        plot(verfine_data(1,:), expData(n,:),'linewidth',2);hold on;
        axis([min(verfine_data(1,:)) max(verfine_data(1,:)) min(expData(n,:))-0.002 max(expData(n,:))+0.002]);
    else
      subplot(3,2,[2 4 6]); hold on;
      plotGMM_3D_P_add_IMP_adj_size([verfine_data(1,:); expData(n,:)], 0.3*expSigma(n-input_dim,n-input_dim,:), [0 0 .8], 3,ellpsiod_magnitude_GMR,ellpsiod_magnitude_GMR_3D);
      axis([min(verfine_data(1,:)) max(verfine_data(1,:)) min(expData(n,:))-0.002 max(expData(n,:))+0.002]);

      % plot ground turth
      hold on;
      plot(verfine_data(1,:), verfine_data(5,:),'r','linewidth',2);grid on;hold on;
    end  
    if n==1
        xlabel('time step','fontsize',16); ylabel(['F_X (N)'],'fontsize',16);grid on;hold on;
    elseif n==2
        xlabel('time step','fontsize',16); ylabel(['F_Y (N)'],'fontsize',16);grid on;hold on;
    elseif n==3
        xlabel('time step','fontsize',16); ylabel(['F_Z (N)'],'fontsize',16);grid on;hold on;
    elseif n==4
        xlabel('time step','fontsize',16); ylabel(['V (m/s)'],'fontsize',16);grid on;hold on;
        legend('variance','regression value','real value');
    end
end
    
figureHandle = gcf;
    %# make all text in the figure to size 14 and bold
    set(findall(figureHandle,'type','text'),'FontName','Times New Roman','FontSize',12)


%% see the input data in 3D with GMM
screen=get(0,'ScreenSize');
figure
set(gcf,'unit','centimeters','position',[3 5 7 5]*3)
 set(gca,'Position',[.15 .15 .8 .75],'FontName','Times New Roman','FontSize',16); 
 
subplot(121); hold on;grid on;view(-78,38);
h=scatter3(Data(1,:),Data(2,:),Data(3,:),30,Data(4,:));
xlabel('Fx'); ylabel('Fy'); zlabel('Fz');
h1 = colorbar('FontName','Times New Roman','FontSize',16);
set(get(h1,'label'),'string','vel maginatude');%
caxis([0 2.2e-3])
subplot(122); hold on;grid on;view(-78,38);
h=scatter3(Data(1,:),Data(2,:),Data(3,:),30,Data(4,:));
h1 = colorbar('FontName','Times New Roman','FontSize',16);
set(get(h1,'label'),'string','vel maginatude');hold on;caxis([0 2.2e-3])
xlabel('Fx'); ylabel('Fy'); zlabel('Fz');
plotGMM_3D_P_add_IMP_adj_size(Mu(1:3,:), Sigma([1:3],[1:3],:), [0 .8 0],...
      6,0.1,ellpsiod_magnitude_GMM_3D);
  
figureHandle = gcf;
%# make all text in the figure to size 14 and bold
set(findall(figureHandle,'type','text'),'FontName','Times New Roman','FontSize',16)


% %% plot 3d out put of the regression
% if mod(nbVar,3)==0
%     screen=get(0,'ScreenSize');
%     figure('position',[screen(:,1:2)+100 screen(:,3:4)/1.3],'name','GMR-demo1');
%     %plot 3D
%      hold on;grid on;
%     plotGMM_3D_P_add_IMP_adj_size(expData([1,2,3],:), expSigma([1,2,3],[1,2,3],:), [0 0 .8], 4,ellpsiod_magnitude_GMR,ellpsiod_magnitude_GMR_3D);
%     axis([min(expData(1,:))-0.01 max(expData(1,:))+0.01 min(expData(2,:))-0.01 max(expData(2,:))+0.01 min(expData(3,:))-0.01 max(expData(3,:))+0.01]);
%     xlabel('x_1','fontsize',16); ylabel('x_2','fontsize',16); zlabel('x_3','fontsize',16);
% 
%     vel_samples = 15; vel_size = 0.8; tol_cutting=0.001;
%      hold on;
%     % Plot Velocities of Reference Trajectories
%         vel_points = expData(:,1:vel_samples:end);
%         U_vel = zeros(size(vel_points,2),1);
%         V_vel = zeros(size(vel_points,2),1);
%         W_vel = zeros(size(vel_points,2),1);
%         for i = 1:size(vel_points, 2)
%             dir_vel    = vel_points(input_dim+1:input_dim+3,i);%show vel
%             U_vel(i,1)   = dir_vel(1);
%             V_vel(i,1)   = dir_vel(2);
%             W_vel(i,1)   = dir_vel(3);
% 
%         end
%         h_vel = quiver3(vel_points(1,:)',vel_points(2,:)',vel_points(3,:)', U_vel, V_vel, W_vel, vel_size, 'Color', 'r', 'LineWidth',2); hold on;
% 
%     vel_samples = 40; vel_size = 0.8; tol_cutting=0.001;
%      hold on;
%     % Plot Velocities of Reference Trajectories
%         vel_points = expData(:,1:vel_samples:end);
%         U_imp = zeros(size(vel_points,2),1);
%         V_imp = zeros(size(vel_points,2),1);
%         W_imp = zeros(size(vel_points,2),1);
%         for i = 1:size(vel_points, 2)
% 
%             dir_imp    = vel_points(input_dim+4:input_dim+6,i);%show vel
%             U_imp(i,1)   = dir_imp(1);
%             V_imp(i,1)   = dir_imp(2);
%             W_imp(i,1)   = dir_imp(3);
%         end
%         
%                 h_imp = quiver3(vel_points(1,:)',vel_points(2,:)',vel_points(3,:)', U_imp, V_imp, W_imp, vel_size, 'Color', 'g', 'LineWidth',2); hold on;
% 
%         grid on;
%         box on;axis equal;
% 
%     % plotGMM_3D_P_add_IMP_adj_size(expData([2,3,4],1:number_windows:end), ellispiod_imp([1,2,3],[1,2,3],:), [0 0 .8], 6 ,ellpsiod_magnitude_GMR,ellpsiod_magnitude_GMR_3D);
%     view(23,16);%axis equal;%axis tight;
% end



% %output the GMR data
% Mu=expData([2,3,4],:);
% Sigma=expSigma([1,2,3],[1,2,3],:);
% x_first=expData([2,3,4],1);
% x_last=expData([2,3,4],end);
% imp=expData([5,6,7],:);

%output the GMM data
% Mu_out=Mu;Sigma_out=Sigma;
% Mu=Mu_out([2,3,4],:);
% Sigma=Sigma_out([2,3,4],[2,3,4],:);
% x_first=expData([2,3,4],1);
% x_last=expData([2,3,4],end);
% imp=Mu_out([5,6,7],:);
% imp_Sigma=Sigma_out([5,6,7],[5,6,7],:);

end

