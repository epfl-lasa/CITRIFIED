function [Mu Sigma Priors] = gmm_gmr_pos2force_3in_3out(demos,verfine_data,nbStates,scale_f_by_p,scale_i_by_p,ellpsiod_magnitude_GMM,ellpsiod_magnitude_GMM_3D,ellpsiod_magnitude_GMR,ellpsiod_magnitude_GMR_3D)
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

%% Training of GMM by EM algorithm, initialized by k-means clustering.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[Priors, Mu, Sigma] = GMM_EM_init_kmeans(Data, nbStates);
[Priors, Mu, Sigma] = GMM_EM(Data, Priors, Mu, Sigma);

number_windows=ceil(length(Data)/nbStates);
for num_ellispiod_imp=1:nbStates 
    Mu_time_plot(:,num_ellispiod_imp)=[demos(1,number_windows*(num_ellispiod_imp-1)+1)];
end

%% Use of GMR to retrieve a generalized version of the data and associated
%% constraints. A sequence of temporal values is used as input, and the 
%% expected distribution is retrieved. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
expData(1:3,:) = verfine_data(2:4,:);
[expData(4:nbVar,:), expSigma] = GMM_GMR(Priors, Mu, Sigma,  expData(1:3,:), [1:3], [4:nbVar]);

%% Plot of the data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
screen=get(0,'ScreenSize');
figure('position',[screen(:,1:2)+100 screen(:,3:4)/1.3],'name','data_GMM-demo1');
%plot 1D
for n=1:nbVar
  subplot(6,2,(n-1)*2+1); hold on;
  plot(Data(1,:), Data(n,:), '*', 'markerSize', 1);%, 'color', [.3 .3 .3],'lineWidth', 3
  axis([min(Data(1,:)) max(Data(1,:)) min(Data(n,:))-0.01 max(Data(n,:))+0.01]);
  xlabel('p','fontsize',16); ylabel(['x_' num2str(n)],'fontsize',16);
end
%plot 3D
subplot(6,2,[2:2:2*(nbVar)]); hold on;
plot3(Data(1,:), Data(2,:), Data(3,:), '*', 'markerSize', 3);%, 'color', [.3 .3 .3]
axis([min(Data(1,:))-0.01 max(Data(1,:))+0.01 min(Data(2,:))-0.01 max(Data(2,:))+0.01 min(Data(3,:))-0.01 max(Data(3,:))+0.01]);
xlabel('x_1','fontsize',16); ylabel('x_2','fontsize',16); zlabel('x_3','fontsize',16);
view(23,16); set(gca,'proj','perspective'); grid on;
  grid on;  axis tight;

%% Plot of the GMM encoding results
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%plot 1D 
for n=1:nbVar
  subplot(6,2,(n-1)*2+1); hold on;
  plotGMM_3D_P_add_IMP_adj_size([Mu(1,:);Mu(n,:)], Sigma([1,n],[1,n],:), [0 .8 0], 1,ellpsiod_magnitude_GMM,ellpsiod_magnitude_GMM_3D);
  axis([min(Data(1,:)) max(Data(1,:)) min(Data(n,:))-0.01 max(Data(n,:))+0.01]);
  xlabel('p','fontsize',16); ylabel(['x_' num2str(n)],'fontsize',16);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%using this part to should orign pose GMM and the force GMM %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% %plot 3D GMM of input : pose
% subplot(6,2,[2:2:2*(nbVar)]); hold on;
% plotGMM_3D_P_add_IMP_adj_size(Mu([2,3,4],:), Sigma([2,3,4],[2,3,4],:), [0 .8 0], 5,ellpsiod_magnitude_GMM,ellpsiod_magnitude_GMM_3D);
% axis([min(Data(2,:))-0.03 max(Data(2,:))+0.03 min(Data(3,:))-0.03 max(Data(3,:))+0.03 min(Data(4,:))-0.01 max(Data(4,:))+0.01]);
% xlabel('x_1','fontsize',16); ylabel('x_2','fontsize',16) ; zlabel('x_3','fontsize',16);
% view(23,16)

%plot 3D GMM of output :force on pose

% for num_ellispiod_imp=1:nbStates  
%     ellispiod_imp(:,:,num_ellispiod_imp)=[abs(Mu(5,num_ellispiod_imp)) 0 0; 0 abs(Mu(6,num_ellispiod_imp)) 0; 0 0 abs(Mu(7,num_ellispiod_imp))];
% end

subplot(6,2,[2:2:2*(nbVar)]); hold on;
plotGMM_3D_P_add_IMP_adj_size(Mu([1,2,3],:), Sigma([4,5,6],[4,5,6],:), [0 .8 0], 5,ellpsiod_magnitude_GMM,ellpsiod_magnitude_GMM_3D);
% plotGMM_3D_P_add_IMP_adj_size(Mu([2,3,4],:), ellispiod_imp([1,2,3],[1,2,3],:), [0 .8 0], 5,ellpsiod_magnitude_GMM,ellpsiod_magnitude_GMM_3D);
% axis([min(Data(2,:))-0.03 max(Data(2,:))+0.03 min(Data(3,:))-0.03 max(Data(3,:))+0.03 min(Data(4,:))-0.01 max(Data(4,:))+0.01]);
xlabel('x_1','fontsize',16); ylabel('x_2','fontsize',16) ; zlabel('x_3','fontsize',16);axis equal;
view(23,16)
%%%%%%%%%%%%%%%%%%using this part to should orign pose GMM and the force GMM %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Plot of the GMR regression results
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%plot 1D
screen=get(0,'ScreenSize');
figure('position',[screen(:,1:2)+100 screen(:,3:4)/1.3],'name','GMR-demo1');
for n=1:3
  subplot(3,2,1+2*(n-1)); hold on;
  plotGMM_3D_P_add_IMP_adj_size([verfine_data(1,:); expData(n+3,:)], expSigma(n,n,:), [0 0 .8], 3,ellpsiod_magnitude_GMR,ellpsiod_magnitude_GMR_3D);
  axis([min(verfine_data(1,:)) max(verfine_data(1,:)) min(expData(n+3,:))-0.01 max(expData(n+3,:))+0.01]);
  xlabel('t','fontsize',16); ylabel(['F_' num2str(n)],'fontsize',16);grid on;
end

%plot 3D
subplot(3,2,[2 4 6]); hold on;grid on;
plotGMM_3D_P_add_IMP_adj_size(expData([1,2,3],:), expSigma([1,2,3],[1,2,3],:), [0 0 .8], 4,ellpsiod_magnitude_GMR,ellpsiod_magnitude_GMR_3D);
axis([min(expData(1,:))-0.01 max(expData(1,:))+0.01 min(expData(2,:))-0.01 max(expData(2,:))+0.01 min(expData(3,:))-0.01 max(expData(3,:))+0.01]);
xlabel('x_1','fontsize',16); ylabel('x_2','fontsize',16); zlabel('x_3','fontsize',16);

%plot 3D imp   %position should be on Mu line, and based on this position
%plot the imp , what's more , the 3*1 imp data can't be plot as a
%ellispoid, so extend it as a 3*3 matrix
% force_number=25;
% number_windows=ceil(length(expData)/force_number);
% for num_ellispiod_imp=1:force_number  
%     ellispiod_imp(:,:,num_ellispiod_imp)=[expData(4,number_windows*(num_ellispiod_imp-1)+1) 0 0; 0 expData(5,number_windows*(num_ellispiod_imp-1)+1) 0; 0 0 expData(6,number_windows*(num_ellispiod_imp-1)+1)];
% end

vel_samples = 15; vel_size = 0.8; tol_cutting=0.001;
subplot(3,2,[2 4 6]); hold on;
% Plot Velocities of Reference Trajectories
    vel_points = expData(:,1:vel_samples:end);
    U = zeros(size(vel_points,2),1);
    V = zeros(size(vel_points,2),1);
    W = zeros(size(vel_points,2),1);
    for i = 1:size(vel_points, 2)
        dir_    = vel_points(4:end,i);%/norm(vel_points(3:end,i))
        U(i,1)   = dir_(1);
        V(i,1)   = dir_(2);
        W(i,1)   = dir_(3);
    end
    h_vel = quiver3(vel_points(1,:)',vel_points(2,:)',vel_points(3,:)', U, V, W, vel_size, 'Color', 'r', 'LineWidth',2); hold on;
    grid on;
    box on;axis equal;

% plotGMM_3D_P_add_IMP_adj_size(expData([2,3,4],1:number_windows:end), ellispiod_imp([1,2,3],[1,2,3],:), [0 0 .8], 6 ,ellpsiod_magnitude_GMR,ellpsiod_magnitude_GMR_3D);
view(23,16);%axis equal;%axis tight;




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

