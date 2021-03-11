function [Mu Sigma Priors] = gmm_gmr_learn_imp_from_pos_out_GMM(demos,nbStates)
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
Data=demos;
nbVar = size(Data,1);

%% Training of GMM by EM algorithm, initialized by k-means clustering.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[Priors, Mu, Sigma] = GMM_EM_init_kmeans(Data, nbStates);
[Priors, Mu, Sigma] = GMM_EM(Data, Priors, Mu, Sigma);

%% Use of GMR to retrieve a generalized version of the data and associated
%% constraints. A sequence of temporal values is used as input, and the 
%% expected distribution is retrieved. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
expData(1,:) = linspace(min(Data(1,:)), max(Data(1,:)), 6100);
[expData(2:nbVar,:), expSigma] = GMM_GMR(Priors, Mu, Sigma,  expData(1,:), [1], [2:nbVar]);

%% Plot of the data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
screen=get(0,'ScreenSize');
figure('position',[screen(:,1:2)+100 screen(:,3:4)/1.3],'name','data_GMM-demo1');
%plot 1D
for n=1:nbVar-1
  subplot(6,2,(n-1)*2+1); hold on;
  plot(Data(1,:), Data(n+1,:), '*', 'markerSize', 1);%, 'color', [.3 .3 .3],'lineWidth', 3
  axis([min(Data(1,:)) max(Data(1,:)) min(Data(n+1,:))-0.01 max(Data(n+1,:))+0.01]);
  xlabel('t','fontsize',16); ylabel(['x_' num2str(n)],'fontsize',16);
end
%plot 3D
subplot(6,2,[2:2:2*(nbVar-1)]); hold on;
plot3(Data(2,:), Data(3,:), Data(4,:), '*', 'markerSize', 3);%, 'color', [.3 .3 .3]
axis([min(Data(2,:))-0.01 max(Data(2,:))+0.01 min(Data(3,:))-0.01 max(Data(3,:))+0.01 min(Data(4,:))-0.01 max(Data(4,:))+0.01]);
xlabel('x_1','fontsize',16); ylabel('x_2','fontsize',16); zlabel('x_3','fontsize',16);
view(-90,12); set(gca,'proj','perspective'); grid on;
  grid on;  axis tight;

%% Plot of the GMM encoding results
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%plot 1D
for n=1:nbVar-1
  subplot(6,2,(n-1)*2+1); hold on;
  plotGMM_3D_P_add_IMP(Mu([1,n+1],:), Sigma([1,n+1],[1,n+1],:), [0 .8 0], 1);
  axis([min(Data(1,:)) max(Data(1,:)) min(Data(n+1,:))-0.01 max(Data(n+1,:))+0.01]);
  xlabel('t','fontsize',16); ylabel(['x_' num2str(n)],'fontsize',16);
end
%plot 3D
subplot(6,2,[2:2:2*(nbVar-1)]); hold on;
plotGMM_3D_P_add_IMP(Mu([2,3,4],:), Sigma([2,3,4],[2,3,4],:), [0 .8 0], 5);
axis([min(Data(2,:))-0.03 max(Data(2,:))+0.03 min(Data(3,:))-0.03 max(Data(3,:))+0.03 min(Data(4,:))-0.01 max(Data(4,:))+0.01]);
xlabel('x_1','fontsize',16); ylabel('x_2','fontsize',16) ; zlabel('x_3','fontsize',16);
view(-90,12)

%% Plot of the GMR regression results
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%plot 1D
screen=get(0,'ScreenSize');
figure('position',[screen(:,1:2)+100 screen(:,3:4)/1.3],'name','GMR-demo1');
for n=1:3
  subplot(6,2,1+2*(n-1)); hold on;
  plotGMM_3D_P_add_IMP(expData([1,n+1],:), expSigma(n,n,:), [0 0 .8], 3);
  axis([min(Data(1,:)) max(Data(1,:)) min(Data(n+1,:))-0.01 max(Data(n+1,:))+0.01]);
  xlabel('t','fontsize',16); ylabel(['x_' num2str(n)],'fontsize',16);
end
for n=4:6
  subplot(6,2,2*(n-3)); hold on;
  plotGMM_3D_P_add_IMP(expData([1,n+1],:), expSigma(n,n,:), [0 0 .8], 3);
  axis([min(Data(1,:)) max(Data(1,:)) min(Data(n+1,:))-0.01 max(Data(n+1,:))+0.01]);
  xlabel('t','fontsize',16); ylabel(['x_' num2str(n)],'fontsize',16);
end
%plot 3D
subplot(6,2,[7 8 9 10 11 12]); hold on;
plotGMM_3D_P_add_IMP(expData([2,3,4],:), expSigma([1,2,3],[1,2,3],:), [0 0 .8], 4);
axis([min(Data(2,:))-0.01 max(Data(2,:))+0.01 min(Data(3,:))-0.01 max(Data(3,:))+0.01 min(Data(4,:))-0.01 max(Data(4,:))+0.01]);
xlabel('x_1','fontsize',16); ylabel('x_2','fontsize',16); zlabel('x_3','fontsize',16);

%plot 3D imp   %position should be on Mu line, and based on this position
%plot the imp , what's more , the 3*1 imp data can't be plot as a
%ellispoid, so extend it as a 3*3 matrix
for num_ellispiod_imp=1:7
    ellispiod_imp(:,:,num_ellispiod_imp)=[expData(5,999*(n-1)+1) 0 0; 0 expData(6,999*(n-1)+1) 0; 0 0 expData(7,999*(n-1)+1)];
end
plotGMM_3D_P_add_IMP(expData([2,3,4],1:999:end), ellispiod_imp([1,2,3],[1,2,3],:), [0 0 .8], 6);
%axis([min(Data(2,:))-0.01 max(Data(2,:))+0.01 min(Data(3,:))-0.01 max(Data(3,:))+0.01 min(Data(4,:))-0.01 max(Data(4,:))+0.01]);
xlabel('x_1','fontsize',16); ylabel('x_2','fontsize',16); zlabel('x_3','fontsize',16);
view(-90,12);%axis equal;%axis tight;



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

