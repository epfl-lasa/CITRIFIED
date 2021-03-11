function [Mu,Sigma,Priors,expData,expSigma,data_mean,data_range] = gmm_gmr_mutil_dimension_normalize_learner(demos,input_dim,output_dim,verfine_data,nbStates,normalize,ellpsiod_magnitude_GMM,ellpsiod_magnitude_GMM_3D,ellpsiod_magnitude_GMR,ellpsiod_magnitude_GMR_3D)
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
    else if nbVar==3+4
            plot_nbVar=3;
        end
    end
end

if mod(output_dim,3)==0
    plot_output=(input_dim+output_dim)/3;
else if output_dim==2 
    plot_output=2;
    else if output_dim==4
            plot_output=3;
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


%% Plot of the data
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

screen=get(0,'ScreenSize');
figure('position',[screen(:,1:2)+100 screen(:,3:4)/1.3],'name','data_GMM-force');
%plot 1D
for n=1:input_dim
  subplot(2,3,n); hold on;
  plot(Data(n,:), Data(n+input_dim+1,:), '*', 'markerSize', 1);%, 'color', [.3 .3 .3],'lineWidth', 3
  axis([min(Data(n,:)) max(Data(n,:)) min(Data(n+input_dim+1,:))-0.01 max(Data(n+input_dim+1,:))+0.01]);
  xlabel('p','fontsize',16); ylabel(['x_' num2str(n)],'fontsize',16);
  subplot(2,3,n+input_dim); hold on;
  plot(Data(n,:), Data(input_dim+1,:), '*', 'markerSize', 1);%, 'color', [.3 .3 .3],'lineWidth', 3
  axis([min(Data(n,:)) max(Data(n,:)) min(Data(input_dim+1,:))-0.01 max(Data(input_dim+1,:))+0.01]);
  xlabel('p','fontsize',16); ylabel(['x_' num2str(n)],'fontsize',16);
end

for n=1:input_dim
  subplot(2,3,n); hold on;
  plotGMM_3D_P_add_IMP_adj_size([Mu(n,:);Mu(n+input_dim+1,:)], Sigma([1,n],[1,n],:), [0 .8 0], 1,ellpsiod_magnitude_GMM,ellpsiod_magnitude_GMM_3D);
  axis([min(Data(n,:)) max(Data(n,:)) min(Data(n+input_dim+1,:))-0.01 max(Data(n+input_dim+1,:))+0.01]);
  xlabel('p','fontsize',16); ylabel(['x_' num2str(n)],'fontsize',16);
  subplot(2,3,n+input_dim); hold on;
  plotGMM_3D_P_add_IMP_adj_size([Mu(n,:);Mu(input_dim+1,:)], Sigma([1,n],[1,n],:), [0 .8 0], 1,ellpsiod_magnitude_GMM,ellpsiod_magnitude_GMM_3D);
  axis([min(Data(n,:)) max(Data(n,:)) min(Data(input_dim+1,:))-0.01 max(Data(input_dim+1,:))+0.01]);
  xlabel('p','fontsize',16); ylabel(['x_' num2str(n)],'fontsize',16);
end

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
screen=get(0,'ScreenSize');
figure('position',[screen(:,1:2)+100 screen(:,3:4)/1.3],'name','GMR-time');
for n=1:input_dim+output_dim
    if n<=input_dim
        subplot(plot_output,3,n); hold on;
        plot(verfine_data(1,:), expData(n,:));
    else
  subplot(plot_output,3,n); hold on;
  plotGMM_3D_P_add_IMP_adj_size([verfine_data(1,:); expData(n,:)], expSigma(n-input_dim,n-input_dim,:), [0 0 .8], 3,ellpsiod_magnitude_GMR,ellpsiod_magnitude_GMR_3D);
%   axis([min(verfine_data(1,:)) max(verfine_data(1,:)) min(expData(n+3,:))-0.01 max(expData(n+3,:))+0.01]);
    end  
xlabel('t','fontsize',16); ylabel(['F_' num2str(n)],'fontsize',16);grid on;
end


%% plot GMM and GMR together
screen=get(0,'ScreenSize');
figure
set(gcf,'unit','centimeters','position',[3 5 7 5]*3)
    set(gca,'Position',[.15 .15 .8 .75],'FontName','Times New Roman','FontSize',16);
%plot 1D
for n=1:input_dim
  subplot(3,3,n); hold on;
  plot(Data(n,:), Data(n+input_dim+1,:), '*', 'markerSize', 1);%, 'color', [.3 .3 .3],'lineWidth', 3
  axis([min(Data(n,:)) max(Data(n,:)) min(Data(n+input_dim+1,:))-0.01 max(Data(n+input_dim+1,:))+0.01]);
   plotGMM_3D_P_add_IMP_adj_size([Mu(n,:);Mu(n+input_dim+1,:)], Sigma([1,n],[1,n],:), [0 .8 0], 1,ellpsiod_magnitude_GMM,ellpsiod_magnitude_GMM_3D);
  axis([min(Data(n,:)) max(Data(n,:)) min(Data(n+input_dim+1,:))-0.01 max(Data(n+input_dim+1,:))+0.01]);

  if n==1
      xlabel('F_X','fontsize',16); ylabel(['Stiffness_X'],'fontsize',16);grid on;
  elseif n==2
      xlabel('F_Y','fontsize',16); ylabel(['Stiffness_Y'],'fontsize',16);grid on;
  else
      xlabel('F_Z','fontsize',16); ylabel(['Stiffness_Z'],'fontsize',16);grid on;
      [~,h_legend] = legend('data','GMM');
      PatchInLegend = findobj(h_legend, 'type', 'patch');
       set(PatchInLegend(1), 'FaceAlpha', 0.2);
%         set(PatchInLegend(2), 'FaceAlpha', 0.4);
  end

end

for n=input_dim+2:input_dim+output_dim
  subplot(3,3,n-1); hold on;
  plot(verfine_data(1,:),verfine_data(n,:),'r');hold on;
  axis([min(verfine_data(1,:)) max(verfine_data(1,:)) min(verfine_data(n,:))-0.01 max(verfine_data(n,:))+0.01]);
  plotGMM_3D_P_add_IMP_adj_size([verfine_data(1,:); expData(n,:)], expSigma(n-input_dim,n-input_dim,:), [0 0 .8], 3,ellpsiod_magnitude_GMR,ellpsiod_magnitude_GMR_3D);
%   axis([min(verfine_data(1,:)) max(verfine_data(1,:)) min(expData(n+3,:))-0.01 max(expData(n+3,:))+0.01]);
    if n==input_dim+2
        xlabel('time step','fontsize',16); ylabel(['Stiffness_X'],'fontsize',16);grid on;
    elseif n==input_dim+3
        xlabel('time step','fontsize',16); ylabel(['Stiffness_Y'],'fontsize',16);grid on;
    else
        xlabel('time step','fontsize',16); ylabel(['Stiffness_Z'],'fontsize',16);grid on;
        legend('groundturth','variance','regression value')
    end
  
end

% learning the linear force to imp regression to compare with the record
% data
for i=1:input_dim
expData_linear(i,:) = linspace(min(Data(i,:)),max(Data(i,:)),size(Data(i,:),2));
end
[expData_linear(input_dim+1:nbVar,:), expSigma_linear] = GMM_GMR(Priors, Mu, Sigma,  expData_linear(1:input_dim,:), [1:input_dim], [input_dim+1:nbVar]);


for n=1:input_dim
  subplot(3,3,n+6); hold on;
  plot(Data(n,:), Data(n+input_dim+1,:), '*', 'markerSize', 1);%, 'color', [.3 .3 .3],'lineWidth', 3
  y = linspace(min(Data(n,:)),max(Data(n,:)),size(expData_linear(n+input_dim+1,:),2));
  plotGMM_3D_P_add_IMP_adj_size([y; expData_linear(n+input_dim+1,:)], expSigma_linear(n+input_dim+1-input_dim,n+input_dim+1-input_dim,:), [0 0 .8], 3,ellpsiod_magnitude_GMR,ellpsiod_magnitude_GMR_3D);
%   axis([min(verfine_data(1,:)) max(verfine_data(1,:)) min(expData(n+3,:))-0.01 max(expData(n+3,:))+0.01]);
    if n==1
        xlabel('time step','fontsize',16); ylabel(['Stiffness_X'],'fontsize',16);grid on;
    elseif n==2
        xlabel('time step','fontsize',16); ylabel(['Stiffness_Y'],'fontsize',16);grid on;
    else
        xlabel('time step','fontsize',16); ylabel(['Stiffness_Z'],'fontsize',16);grid on;
        legend('groundturth','variance','regression value')
    end
  
end

figureHandle = gcf;
%# make all text in the figure to size 14 and bold
set(findall(figureHandle,'type','text'),'FontName','Times New Roman','FontSize',16)


%% plot GMM and GMR together only z direction
screen=get(0,'ScreenSize');
figure
set(gcf,'unit','centimeters','position',[3 5 7 5]*3)
    set(gca,'Position',[.15 .15 .8 .75],'FontName','Times New Roman','FontSize',16);

 n=input_dim+output_dim;
  subplot(212); hold on;
  plot(verfine_data(1,:),verfine_data(n,:),'r');hold on;
  axis([min(verfine_data(1,:)) max(verfine_data(1,:)) min(verfine_data(n,:))-0.01 max(verfine_data(n,:))+0.01]);
  plotGMM_3D_P_add_IMP_adj_size([verfine_data(1,:); expData(n,:)], expSigma(n-input_dim,n-input_dim,:), [0 0 .8], 3,ellpsiod_magnitude_GMR,ellpsiod_magnitude_GMR_3D);
%   axis([min(verfine_data(1,:)) max(verfine_data(1,:)) min(expData(n+3,:))-0.01 max(expData(n+3,:))+0.01]);
    if n==input_dim+2
        xlabel('F_x','fontsize',16); ylabel(['Stiffness_X'],'fontsize',16);grid on;
    elseif n==input_dim+3
        xlabel('F_y','fontsize',16); ylabel(['Stiffness_Y'],'fontsize',16);grid on;
    else
        xlabel('F_z','fontsize',16); ylabel(['Stiffness_Z'],'fontsize',16);grid on;
        legend('groundturth','variance','regression value')
    end
  
% learning the linear force to imp regression to compare with the record
% data
for i=1:input_dim
expData_linear(i,:) = linspace(min(Data(i,:)),max(Data(i,:)),size(Data(i,:),2));
end
[expData_linear(input_dim+1:nbVar,:), expSigma_linear] = GMM_GMR(Priors, Mu, Sigma,  expData_linear(1:input_dim,:), [1:input_dim], [input_dim+1:nbVar]);

n=3
  subplot(211); hold on;
  plot(Data(n,:), Data(n+input_dim+1,:), '*', 'markerSize', 1);%, 'color', [.3 .3 .3],'lineWidth', 3
  y = linspace(min(Data(n,:)),max(Data(n,:)),size(expData_linear(n+input_dim+1,:),2));
  plotGMM_3D_P_add_IMP_adj_size([y; expData_linear(n+input_dim+1,:)], expSigma_linear(n+input_dim+1-input_dim,n+input_dim+1-input_dim,:), [0 0 .8], 3,ellpsiod_magnitude_GMR,ellpsiod_magnitude_GMR_3D);
%   axis([min(verfine_data(1,:)) max(verfine_data(1,:)) min(expData(n+3,:))-0.01 max(expData(n+3,:))+0.01]);
    if n==1
        xlabel('time step','fontsize',16); ylabel(['Stiffness_X'],'fontsize',16);grid on;
    elseif n==2
        xlabel('time step','fontsize',16); ylabel(['Stiffness_Y'],'fontsize',16);grid on;
    else
        xlabel('time step','fontsize',16); ylabel(['Stiffness_Z'],'fontsize',16);grid on;
        legend('groundturth','variance','regression value')
    end
  
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

