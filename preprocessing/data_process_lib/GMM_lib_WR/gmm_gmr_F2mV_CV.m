function [Mu,Sigma,Priors,expData,expSigma,data_mean,data_std,MSE] = gmm_gmr_F2mV_CV(demos,input_dim,output_dim,verfine_data,nbStates,normalize_or_not,ellpsiod_magnitude_GMM,ellpsiod_magnitude_GMM_3D,ellpsiod_magnitude_GMR,ellpsiod_magnitude_GMR_3D,plot_on)
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

%% normalize the data for better learning
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
vector_input_magni=[];
vector_output_magni=[];
if normalize_or_not==1

    mean_input=mean(Data(1:3,:),2);
    std_input=std(Data(1:3,:)');
    mean_output=mean(Data(4,:),2);
    std_output=std(Data(4,:)');
    
    Data1(1,:)=(Data(1,:)-mean_input(1))/std_input(1);
    Data1(2,:)=(Data(2,:)-mean_input(2))/std_input(2);
    Data1(3,:)=(Data(3,:)-mean_input(3))/std_input(3);
    Data1(4,:)=(Data(4,:)-mean_output(1))/std_output(1);
%     Data1(5,:)=(Data(5,:)-mean_output(2))/std_output(2);
%     Data1(6,:)=(Data(6,:)-mean_output(3))/std_output(3);
    
    Data2(1:3,:)=(Data(1:3,:)-mean_input)/sqrt(std_input(1)^2+std_input(2)^2+std_input(3)^2);
    Data2(4,:)=(Data(4,:)-mean_output)/std_output(1);
    
    if plot_on==1
    figure
    subplot(211)
    plot3(Data1(1,:),Data1(2,:),Data1(3,:),'ro');hold on;grid on;
    plot3(Data2(1,:),Data2(2,:),Data2(3,:),'bo');hold on;grid on;
    title('input');
    subplot(212)
    plot(1:length(Data1(4,:)),Data1(4,:),'r*');hold on;grid on;
    plot(1:length(Data2(4,:)),Data2(4,:),'b*');hold on;grid on;
    title('output');
    end
    
    Data=Data2;
    verfine_data(2:4,:)=(verfine_data(2:4,:)-mean_input)/sqrt(std_input(1)^2+std_input(2)^2+std_input(3)^2);
    verfine_data(5,:)=(verfine_data(5,:)-mean_output)/std_output(1);
    clear Data1 Data2
    
    data_mean=[mean_input;mean_output];
    data_std=[std_input std_output]';
else
    data_mean=0;
    data_std=1;
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

MSE=immse(expData(input_dim+1:nbVar,:),verfine_data(input_dim+2:nbVar+1,:));

%% Plot of the GMM learning results
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if plot_on==1
%% plot GMM and GMR
screen=get(0,'ScreenSize');
figure('position',[screen(:,1:2)+100 screen(:,3:4)/1.3],'name','data_GMM-force');
%plot 1D


for n=1:input_dim
  subplot(1,3,n); hold on;
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
for n=1:input_dim
  subplot(1,3,n); hold on; grid on;
  plot(Data(n,:), Data(input_dim+1,:), 'b*', 'markerSize', 1);%, 'color', [.3 .3 .3],'lineWidth', 3
end

%% plot GMM in 6x6
screen=get(0,'ScreenSize');
figure('position',[screen(:,1:2)+100 screen(:,3:4)/1.3],'name','data_GMM-force');
%plot 1D
for n=1:input_dim+output_dim
    for m=1:input_dim+output_dim
      subplot(4,4,(n-1)*4+m); hold on; grid on;
      plot(Data(n,:), Data(m,:), 'b*', 'markerSize', 1);%, 'color', [.3 .3 .3],'lineWidth', 3
    end
end
for n=1:input_dim+output_dim
    for m=1:input_dim+output_dim 
      subplot(4,4,(n-1)*4+m); hold on;
      plotGMM_3D_P_add_IMP_adj_size([Mu(n,:);Mu(m,:)], Sigma([n,m],[n,m],:), [0 .8 0],...
          1,0.1,ellpsiod_magnitude_GMM_3D);
      axis([min(Data(n,:)) max(Data(n,:)) min(Data(m,:))-0.002 max(Data(m,:))+0.002]);
      if n==1
          if m==1
            xlabel('F_X','fontsize',10); ylabel(['F_X'],'fontsize',10);grid on;
          elseif m==2
              xlabel('F_X','fontsize',10); ylabel(['F_Y'],'fontsize',10);grid on;
          elseif m==3
              xlabel('F_X','fontsize',10); ylabel(['F_Z'],'fontsize',10);grid on;
          elseif m==4
              xlabel('F_X','fontsize',10); ylabel(['I_X'],'fontsize',10);grid on;
%           elseif m==5
%               xlabel('F_X','fontsize',10); ylabel(['I_Y'],'fontsize',10);grid on;
%           elseif m==6
%               xlabel('F_X','fontsize',10); ylabel(['I_Z'],'fontsize',10);grid on;
          end
        elseif n==2
            if m==1
            xlabel('F_Y','fontsize',10); ylabel(['F_X'],'fontsize',10);grid on;
          elseif m==2
              xlabel('F_Y','fontsize',10); ylabel(['F_Y'],'fontsize',10);grid on;
          elseif m==3
              xlabel('F_Y','fontsize',10); ylabel(['F_Z'],'fontsize',10);grid on;
          elseif m==4
              xlabel('F_Y','fontsize',10); ylabel(['I_X'],'fontsize',10);grid on;
%           elseif m==5
%               xlabel('F_Y','fontsize',10); ylabel(['I_Y'],'fontsize',10);grid on;
%           elseif m==6
%               xlabel('F_Y','fontsize',10); ylabel(['I_Z'],'fontsize',10);grid on;
          end
        elseif n==3
            if m==1
            xlabel('F_Z','fontsize',10); ylabel(['F_X'],'fontsize',10);grid on;
          elseif m==2
              xlabel('F_Z','fontsize',10); ylabel(['F_Y'],'fontsize',10);grid on;
          elseif m==3
              xlabel('F_Z','fontsize',10); ylabel(['F_Z'],'fontsize',10);grid on;
          elseif m==4
              xlabel('F_Z','fontsize',10); ylabel(['I_X'],'fontsize',10);grid on;
%           elseif m==5
%               xlabel('F_Z','fontsize',10); ylabel(['I_Y'],'fontsize',10);grid on;
%           elseif m==6
%               xlabel('F_Z','fontsize',10); ylabel(['I_Z'],'fontsize',10);grid on;
            end
      elseif n==4
          if m==1
            xlabel('I_X','fontsize',10); ylabel(['F_X'],'fontsize',10);grid on;
          elseif m==2
              xlabel('I_X','fontsize',10); ylabel(['F_Y'],'fontsize',10);grid on;
          elseif m==3
              xlabel('I_X','fontsize',10); ylabel(['F_Z'],'fontsize',10);grid on;
          elseif m==4
              xlabel('I_X','fontsize',10); ylabel(['I_X'],'fontsize',10);grid on;
%           elseif m==5
%               xlabel('I_X','fontsize',10); ylabel(['I_Y'],'fontsize',10);grid on;
%           elseif m==6
%               xlabel('I_X','fontsize',10); ylabel(['I_Z'],'fontsize',10);grid on;
            end
%       elseif n==5
%           if m==1
%             xlabel('I_Y','fontsize',10); ylabel(['F_X'],'fontsize',10);grid on;
%           elseif m==2
%               xlabel('I_Y','fontsize',10); ylabel(['F_Y'],'fontsize',10);grid on;
%           elseif m==3
%               xlabel('I_Y','fontsize',10); ylabel(['F_Z'],'fontsize',10);grid on;
%           elseif m==4
%               xlabel('I_Y','fontsize',10); ylabel(['I_X'],'fontsize',10);grid on;
%           elseif m==5
%               xlabel('I_Y','fontsize',10); ylabel(['I_Y'],'fontsize',10);grid on;
%           elseif m==6
%               xlabel('I_Y','fontsize',10); ylabel(['I_Z'],'fontsize',10);grid on;
%             end
%       elseif n==6
%           if m==1
%             xlabel('I_Z','fontsize',10); ylabel(['F_X'],'fontsize',10);grid on;
%           elseif m==2
%               xlabel('I_Z','fontsize',10); ylabel(['F_Y'],'fontsize',10);grid on;
%           elseif m==3
%               xlabel('I_Z','fontsize',10); ylabel(['F_Z'],'fontsize',10);grid on;
%           elseif m==4
%               xlabel('I_Z','fontsize',10); ylabel(['I_X'],'fontsize',10);grid on;
%           elseif m==5
%               xlabel('I_Z','fontsize',10); ylabel(['I_Y'],'fontsize',10);grid on;
%           elseif m==6
%               xlabel('I_Z','fontsize',10); ylabel(['I_Z'],'fontsize',10);grid on;
%             end
      end
    grid on;
    end
end

% X = [Data(1,:),Data(2,:),Data(3,:),Data(4,:)];
% varNames = {'F_X'; 'F_Y'; 'F_Z'; 'V'};
% figure
% gplotmatrix(X,[],Cylinders,['c' 'b' 'm' 'g' 'r'],[],[],false);
% text([.08 .24 .43 .66 .83], repmat(-.1,1,5), varNames, 'FontSize',8);
% text(repmat(-.12,1,5), [.86 .62 .41 .25 .02], varNames, 'FontSize',8, 'Rotation',90);

%% Plot of the GMR regression results
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%plot 1D
screen=get(0,'ScreenSize');
figure('position',[screen(:,1:2)+100 screen(:,3:4)/1.3],'name','GMR-time');
for n=1:input_dim+output_dim
    if n<=input_dim
        subplot(3,2,n*2-1); hold on;
        plot(1:length(expData(n,:)), expData(n,:),'*','Linewidth',1);
    else
    subplot(3,2,[2 4 6]); hold on;
    plotGMM_3D_P_add_IMP_adj_size([1:length(expData(n,:)); expData(n,:)], expSigma(n-input_dim,n-input_dim,:), [0 0 .8], 3,ellpsiod_magnitude_GMR,ellpsiod_magnitude_GMR_3D);
    plot(1:length(verfine_data(n+1,:)), verfine_data(n+1,:),'r*','Linewidth',1);
  %   axis([min(verfine_data(1,:)) max(verfine_data(1,:)) min(expData(n+3,:))-0.01 max(expData(n+3,:))+0.01]);
    end  
xlabel('t','fontsize',16); ylabel(['F_' num2str(n)],'fontsize',16);grid on;
end

else
end


end

