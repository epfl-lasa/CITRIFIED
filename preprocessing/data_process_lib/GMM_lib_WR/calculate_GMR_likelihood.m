function [min_like,max_like] = calculate_GMR_likelihood(verfine_data,input_dataset,Priors, Mu, Sigma,ellpsiod_magnitude_GMR,ellpsiod_magnitude_GMR_3D)

expData(1:3,:) = verfine_data(2:4,:);
nbVar = size(expData,1)*2;
[expData(4:nbVar,:), expSigma] = GMM_GMR(Priors, Mu, Sigma,  expData(1:3,:), [1:3], [4:nbVar]);

%% get all likeli
%% %%%% 2D %%%%  calcuated likelihood for input point input_new_x in 2D
Mu_like=Mu ;Sigma_like=Sigma ; Priors_like=Priors ;
% Mu_like=Mu_input ;Sigma_like=Sigma_input ; Priors_like=Priors_input ;
likelihood_GMM=0;likelihood_GMM_grid=[];
grid_number=100;
x = linspace(min(expData(1,:))-0.1,max(expData(1,:))+0.1,grid_number);
y = linspace(min(expData(2,:))-0.1,max(expData(2,:))+0.1,grid_number);
z = linspace(min(expData(3,:))-0.1,max(expData(3,:))+0.1,grid_number);
[X,Z] = meshgrid(x,z);
d = size(Sigma_like,1)/2-1;

for i=1:grid_number
for j=1:grid_number
  for k=1:length(Mu_like)
%           tmp = [x(i);z(j)]' - Mu_like([1 3],k)';
%           prob = sum((tmp/Sigma_like([1 3],[1 3],k)).*tmp, 2);
          tmp = [x(i);z(j)] - Mu_like([1 3],k);
          prob = (tmp'*inv(Sigma_like([1 3],[1 3],k)))*tmp;
          Pxi(:,k) = exp(-0.5*prob) / sqrt((2*pi)^(2*d) * (abs(det(Sigma_like([1 3],[1 3],k)))+realmin));
        if k==1
            likelihood_GMM=Pxi(:,k)*Priors_like(k);
        else
          likelihood_GMM=likelihood_GMM+(Pxi(:,k)*Priors_like(k));
        end
  end
%   if likelihood_GMM>100
%       likelihood_GMM=100
%   end
  likelihood_GMM_grid(i,j)=likelihood_GMM;
%   plot3(x(i),z(j),likelihood_GMM,'b*');hold on;
end
end
% xlabel('x1');ylabel('x3');
likelihood_GMM_grid=likelihood_GMM_grid';%it must run, cause by the inner of 3D surface plot
% likelihood_GMM_grid=(likelihood_GMM_grid-min(likelihood_GMM_grid(:)))/(max(likelihood_GMM_grid(:))-min(likelihood_GMM_grid(:)));
figure
surf(X,Z,likelihood_GMM_grid);
figure
for i=1:length(input_dataset)
plot(input_dataset{i}(1,:),input_dataset{i}(3,:),'r-','linewidth',2);hold on;
end
contour(X,Z,likelihood_GMM_grid,[min(likelihood_GMM_grid(:)): 1:max(likelihood_GMM_grid(:))-30],'--');
% plot3(X,Z,likelihood_GMM_grid)
xlabel('x1');ylabel('x3');

%% %%%% 3D %%%%  calcuated likelihood for input point input_new_x in 2D
Mu_like=Mu ;Sigma_like=Sigma ; Priors_like=Priors ;
likelihood_GMM=0;likelihood_GMM_grid=[];likelihood_GMM_grid_l=[];likelihood_GMM_grid_plot=[];
grid_number=100;
x = linspace(min(expData(1,:))-0.1,max(expData(1,:))+0.1,grid_number);
y = linspace(min(expData(2,:))-0.1,max(expData(2,:))+0.1,grid_number);
z = linspace(min(expData(3,:))-0.1,max(expData(3,:))+0.1,grid_number);
[X,Z] = meshgrid(x,z);
d = size(Sigma_like,1)/2-1;

for i=1:grid_number
for j=1:grid_number
for l=1:grid_number
  for k=1:length(Mu_like)
%           tmp = [x(i);z(j)]' - Mu_like([1 3],k)';
%           prob = sum((tmp/Sigma_like([1 3],[1 3],k)).*tmp, 2);
          tmp = [x(i);y(j);z(l)] - Mu_like([1:3],k);
          prob = (tmp'*inv(Sigma_like([1:3],[1:3],k)))*tmp;
          Pxi(:,k) = exp(-0.5*prob) / sqrt((2*pi)^(2*d) * (abs(det(Sigma_like([1:3],[1:3],k)))+realmin));
        if k==1
            likelihood_GMM=Pxi(:,k)*Priors_like(k);
        else
          likelihood_GMM=likelihood_GMM+(Pxi(:,k)*Priors_like(k));
        end
  end

  likelihood_GMM_grid(i,j,l)=likelihood_GMM;
end
end
end
likelihood_GMM_grid_plot=likelihood_GMM_grid_plot';%it must run, cause by the inner of 3D surface plot

figure
for i=1:grid_number
    for j=1:grid_number
    likelihood_GMM_grid_plot_XZ(i,j)=mean(likelihood_GMM_grid(i,:,j));
    end
end
surf(X,Z,likelihood_GMM_grid_plot_XZ');

figure
subplot(311) %% X-Z plant
for i=1:length(input_dataset)
plot(input_dataset{i}(1,:),input_dataset{i}(3,:),'r-','linewidth',2);hold on;
end
contour(X,Z,likelihood_GMM_grid_plot_XZ');%,[min(likelihood_GMM_grid_plot(:)): 1:max(likelihood_GMM_grid_plot(:))-30],'--');
xlabel('x1');ylabel('x3');

subplot(312) %% X-Y plant
for i=1:length(input_dataset)
plot(input_dataset{i}(1,:),input_dataset{i}(2,:),'r-','linewidth',2);hold on;
end
for i=1:grid_number
    for j=1:grid_number
    likelihood_GMM_grid_plot_XY(i,j)=mean(likelihood_GMM_grid(i,j,:));
    end
end
[X,Y] = meshgrid(x,y);
contour(X,Y,likelihood_GMM_grid_plot_XY');%,[min(likelihood_GMM_grid_plot(:)): 1:max(likelihood_GMM_grid_plot(:))-30],'--');
xlabel('x1');ylabel('x2');

subplot(313) %% Y-Z plant
for i=1:length(input_dataset)
plot(input_dataset{i}(2,:),input_dataset{i}(3,:),'r-','linewidth',2);hold on;
end
for i=1:grid_number
    for j=1:grid_number
    likelihood_GMM_grid_plot_YZ(i,j)=mean(likelihood_GMM_grid(:,i,j));
    end
end
[Y,Z] = meshgrid(y,z);
contour(Y,Z,likelihood_GMM_grid_plot_YZ');%,[min(likelihood_GMM_grid_plot(:)): 1:max(likelihood_GMM_grid_plot(:))-30],'--');
xlabel('x2');ylabel('x3');



%% get inputpoint likeli

min_like=min(likelihood_GMM_grid(:));
max_like=max(likelihood_GMM_grid(:));

Mu_like=Mu ;Sigma_like=Sigma ; Priors_like=Priors ;
likelihood_GMM=0;
d = size(Sigma_like,1)/2;
 figure
for i=1:length(expData)
  for k=1:length(Mu_like)
          tmp = expData(1:3,i) - Mu_like(1:3,k);
          prob = (tmp'*inv(Sigma_like(1:3,1:3,k)))*tmp;
          Pxi(:,k) = exp(-0.5*prob) / sqrt((2*pi)^(2*d) * (abs(det(Sigma_like(1:3,1:3,k)))+realmin));
        if k==1
            likelihood_GMM=Pxi(:,k)*Priors_like(k);
        else
          likelihood_GMM=likelihood_GMM+(Pxi(:,k)*Priors_like(k));
        end
  end
 likelihood_GMM=(likelihood_GMM-min_like)/(max_like-min_like);

%  if likelihood_GMM>0.1
%      likelihood_GMM=0.98;
%  end
 

 plot(i,likelihood_GMM,'r-*');hold on;
 
  expData(4:nbVar,i)=expData(4:nbVar,i)*likelihood_GMM;
  expSigma(:,:,i)=expSigma(:,:,i)*likelihood_GMM;
end
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

%% plot 2d gmr with likeli together
figure
hold on;grid on;
plotGMM_2D_P_add_IMP_adj_size(expData([1,3],:), expSigma([1,3],[1,3],:), [0 0 .8], 4,ellpsiod_magnitude_GMR,ellpsiod_magnitude_GMR_3D);

xlabel('x_1','fontsize',16); ylabel('x_3','fontsize',16); 

%plot 3D imp   %position should be on Mu line, and based on this position
%plot the imp , what's more , the 3*1 imp data can't be plot as a
%ellispoid, so extend it as a 3*3 matrix
% force_number=25;
% number_windows=ceil(length(expData)/force_number);
% for num_ellispiod_imp=1:force_number  
%     ellispiod_imp(:,:,num_ellispiod_imp)=[expData(4,number_windows*(num_ellispiod_imp-1)+1) 0 0; 0 expData(5,number_windows*(num_ellispiod_imp-1)+1) 0; 0 0 expData(6,number_windows*(num_ellispiod_imp-1)+1)];
% end

vel_samples = 15; vel_size = 0.8; tol_cutting=0.001;
 hold on;
% Plot Velocities of Reference Trajectories
    vel_points = expData(:,1:vel_samples:end);
    U = zeros(size(vel_points,2),1);

    W = zeros(size(vel_points,2),1);
    for i = 1:size(vel_points, 2)
        dir_    = vel_points([4 6],i);%/norm(vel_points(3:end,i))
        U(i,1)   = dir_(1);

        W(i,1)   = dir_(2);
    end
    h_vel = quiver(vel_points(1,:)',vel_points(3,:)', U, V, vel_size, 'Color', 'r', 'LineWidth',2); hold on;
    grid on;
    axis equal;
hold on;
contour(X,Z,likelihood_GMM_grid_plot_XZ');%,[min(likelihood_GMM_grid_plot(:)): 2:max(likelihood_GMM_grid_plot(:))],'--');

a=1;
end

