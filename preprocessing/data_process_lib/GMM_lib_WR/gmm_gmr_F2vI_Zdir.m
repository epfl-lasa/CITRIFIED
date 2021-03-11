function [Mu,Sigma,Priors,expData,expSigma,data_mean,data_range] = gmm_gmr_F2vI_Zdir(demos,input_dim,output_dim,verfine_data,nbStates,normalize,ellpsiod_magnitude_GMM,ellpsiod_magnitude_GMM_3D,ellpsiod_magnitude_GMR,ellpsiod_magnitude_GMR_3D)
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
expData(1:input_dim,:) = verfine_data(2,:);
[expData(input_dim+1:nbVar,:), expSigma] = GMM_GMR(Priors, Mu, Sigma,  expData(1:input_dim,:), [1:input_dim], [input_dim+1:nbVar]);


%% Plot of the data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
screen=get(0,'ScreenSize');
figure('position',[screen(:,1:2)+100 screen(:,3:4)/1.3],'name','data_GMM-time');
%plot 1D
for n=1:nbVar
  subplot(2,1,n); hold on;
  plot(demos(1,:), Data(n,:)*data_range(n)+data_mean(n), '*', 'markerSize', 1);%, 'color', [.3 .3 .3],'lineWidth', 3
  axis([min(demos(1,:)) max(demos(1,:)) min(Data(n,:)*data_range(n)+data_mean(n))-0.01 max(Data(n,:)*data_range(n)+data_mean(n))+0.01]);
  xlabel('p','fontsize',16); ylabel(['x_' num2str(n)],'fontsize',16);
end



%% Plot of the GMM encoding results
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
screen=get(0,'ScreenSize');
figure('position',[screen(:,1:2)+100 screen(:,3:4)/1.3],'name','data_GMM-force');
%plot 1D
for n=1:input_dim
  subplot(2,1,1); hold on;
  plot(Data(1,:), Data(2,:), '*', 'markerSize', 1);%, 'color', [.3 .3 .3],'lineWidth', 3
  axis([min(Data(1,:)) max(Data(1,:)) min(Data(2,:))-0.01 max(Data(2,:))+0.01]);
  xlabel('p','fontsize',16); ylabel(['x_' num2str(n)],'fontsize',16);
end

for n=1:input_dim
  subplot(2,1,1); hold on;
  plotGMM_3D_P_add_IMP_adj_size([Mu(n,:);Mu(n+1,:)], Sigma([1,n],[1,n],:), [0 .8 0], 1,ellpsiod_magnitude_GMM,ellpsiod_magnitude_GMM_3D);
  axis([min(Data(n,:)) max(Data(n,:)) min(Data(n+1,:))-0.01 max(Data(n+1,:))+0.01]);
  xlabel('p','fontsize',16); ylabel(['x_' num2str(n)],'fontsize',16);
end

%% Plot of the GMR regression results
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%plot 1D
screen=get(0,'ScreenSize');
figure('position',[screen(:,1:2)+100 screen(:,3:4)/1.3],'name','GMR-time');
for n=1:input_dim+output_dim
    if n<=input_dim
        subplot(2,1,n); hold on;
        plot(verfine_data(1,:), expData(n,:));
    else
  subplot(2,1,n); hold on;
  plot(verfine_data(1,:), verfine_data(3,:),'r');
  plotGMM_3D_P_add_IMP_adj_size([verfine_data(1,:); expData(n,:)], expSigma(n-input_dim,n-input_dim,:), [0 0 .8], 3,ellpsiod_magnitude_GMR,ellpsiod_magnitude_GMR_3D);
%   axis([min(verfine_data(1,:)) max(verfine_data(1,:)) min(expData(n+3,:))-0.01 max(expData(n+3,:))+0.01]);
    end  
xlabel('t','fontsize',16); ylabel(['F_' num2str(n)],'fontsize',16);grid on;
end

%% plot GMM and GMR together only z direction
screen=get(0,'ScreenSize');
figure
set(gcf,'unit','centimeters','position',[3 5 7 5]*3)
    set(gca,'Position',[.15 .15 .8 .75],'FontName','Times New Roman','FontSize',16);

n=2;
  subplot(212); hold on;
    plotGMM_3D_P_add_IMP_adj_size([verfine_data(1,:); expData(n,:)], expSigma(n-input_dim,n-input_dim,:), [0 0 .8], 3,ellpsiod_magnitude_GMR,ellpsiod_magnitude_GMR_3D);
%   axis([min(verfine_data(1,:)) max(verfine_data(1,:)) min(expData(n+3,:))-0.01 max(expData(n+3,:))+0.01]);
plot(verfine_data(1,:),verfine_data(3,:),'g','lineWidth', 3);hold on;
  axis([min(verfine_data(1,:)) max(verfine_data(1,:)) min(verfine_data(3,:))-0.01 max(verfine_data(3,:))+0.01]);

    if n==input_dim+2
        xlabel('F_x','fontsize',16); ylabel(['Stiffness_X'],'fontsize',16);grid on;
    elseif n==input_dim+3
        xlabel('F_y','fontsize',16); ylabel(['Stiffness_Y'],'fontsize',16);grid on;
    else
        xlabel('time step','fontsize',16); ylabel(['Impedance(\Omega)'],'fontsize',16);grid on;
        legend({'groundturth','variance','regression value'}, 'Location', 'Best')
    end
  
% learning the linear force to imp regression to compare with the record
% data
for i=1:input_dim
expData_linear(i,:) = linspace(min(Data(i,:)),max(Data(i,:)),size(Data(i,:),2));
end
[expData_linear(input_dim+1:nbVar,:), expSigma_linear] = GMM_GMR(Priors, Mu, Sigma,  expData_linear(1:input_dim,:), [1:input_dim], [input_dim+1:nbVar]);

n=1;
  subplot(211); hold on;
  
  y = linspace(min(Data(n,:)),max(Data(n,:)),size(expData_linear(n+1,:),2));
  plotGMM_3D_P_add_IMP_adj_size([y; expData_linear(n+1,:)], expSigma_linear(1,1,:), [0 0 .8], 3,ellpsiod_magnitude_GMR,ellpsiod_magnitude_GMR_3D);
%   axis([min(verfine_data(1,:)) max(verfine_data(1,:)) min(expData(n+3,:))-0.01 max(expData(n+3,:))+0.01]);
plot(Data(n,:), Data(n+1,:), 'k*', 'markerSize', 1);%, 'color', [.3 .3 .3],'lineWidth', 3

        xlabel('Force(N)','fontsize',16); ylabel(['Impedance(\Omega)'],'fontsize',16);grid on;
        legend({'variance','regression value','groundturth'}, 'Location', 'Best')

  
figureHandle = gcf;
%# make all text in the figure to size 14 and bold
set(findall(figureHandle,'type','text'),'FontName','Times New Roman','FontSize',16)


end

