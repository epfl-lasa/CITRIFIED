function [Mu,Sigma,Priors,expData,expSigma,data_mean,data_range] = SEDS_mutil_dimension_normalize_learner(demos,input_dim,output_dim,verfine_data,verfine_time,nbStates,normalize,ellpsiod_magnitude_GMM,ellpsiod_magnitude_GMM_3D,ellpsiod_magnitude_GMR,ellpsiod_magnitude_GMR_3D,phase,simulation_seds,pos_data_serial_norm_time)
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


%% seds
% User Parameters and Setting
    % Pre-processing
    dt = 1; %The time step of the demonstrations
    tol_cutting = 1.5e-6; % A threshold on velocity that will be used for trimming demos

    % Training parameters
   

    % A set of options that will be passed to the solver. Please type 
    % 'doc preprocess_demos' in the MATLAB command window to get detailed
    % information about other possible options.
    options.tol_mat_bias = 10^-6; % A very small positive scalar to avoid
                                  % instabilities in Gaussian kernel [default: 10^-15]

    options.display = 1;          % An option to control whether the algorithm
                                  % displays the output of each iterations [default: true]

    options.tol_stopping=10^-10;  % A small positive scalar defining the stoppping
                                  % tolerance for the optimization solver [default: 10^-10]

    options.max_iter = 500;       % Maximum number of iteration for the solver [default: i_max=1000]

    options.objective = 'likelihood';    % 'likelihood': use likelihood as criterion to
                                  % optimize parameters of GMM
                                  % 'mse': use mean square error as criterion to
                                  % optimize parameters of GMM
                                  % 'direction': minimize the angle between the
                                  % estimations and demonstrations (the velocity part)
                                  % to optimize parameters of GMM                              
                                  % [default: 'mse']

  seds_input=[];
    for i=1:size(pos_data_serial_norm_time,2)
        if i < verfine_time
            seds_input{i}=pos_data_serial_norm_time{phase,i}(2:4,:);
            t{i}=pos_data_serial_norm_time{phase,i}(1,:);
        elseif i>verfine_time
            seds_input{i-1}=pos_data_serial_norm_time{phase,i}(2:4,:);
            t{i-1}=pos_data_serial_norm_time{phase,i}(1,:);
        end
    end
    [x0 , xT, Data_for_simu, index] = preprocess_demos(seds_input,t,tol_cutting); %preprocessing datas
    
    [Priors_0, Mu_0, Sigma_0] = initialize_SEDS(Data_for_simu,nbStates); %finding an initial guess for GMM's parameter
    [Priors Mu Sigma]=SEDS_Solver(Priors_0,Mu_0,Sigma_0,Data_for_simu,options); %running SEDS optimization solver

    
%     Priors_P2VIdir=Priors;
%     Mu_P2VIdir=Mu;
%     Sigma_P2VIdir=Sigma;
%     nbVar = size(Mu_P2VIdir,1);
    
%% Use of GMR to retrieve a generalized version of the data and associated
%% constraints. A sequence of temporal values is used as input, and the 
%% expected distribution is retrieved. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
verfine_data(2:end,:)=verfine_data(2:end,:)-verfine_data(2:end,end);% make sure all data is using orign as target
expData(1:input_dim,:) = verfine_data(2:end,:);
[expData(input_dim+1:nbVar,:), expSigma] = GMM_GMR(Priors, Mu, Sigma,  expData(1:input_dim,:), [1:input_dim], [input_dim+1:nbVar]);
% [expData(input_dim+1:nbVar,:), expSigma] = GMM_GMR(Priors_0, Mu_0, Sigma_0,  expData(1:input_dim,:), [1:input_dim], [input_dim+1:nbVar]);

    
    if simulation_seds==1
        %% Simulation
        % A set of options that will be passed to the Simulator. Please type 
        % 'doc preprocess_demos' in the MATLAB command window to get detailed
        % information about each option.
        opt_sim.dt = 0.2;
        opt_sim.i_max = 3000;
        opt_sim.tol = 0.00001;
        d = size(Data_for_simu,1)/2; %dimension of data
        x0_all = Data_for_simu(1:d,index(1:end-1)); %finding initial points of all demonstrations
        fn_handle = @(x) GMR(Priors,Mu,Sigma,x,1:d,d+1:2*d);
        [x xd]=Simulation(x0_all,[],fn_handle,opt_sim); %running the simulator
        
        % plotting the result
        figure('name','Results from Simulation','position',[265   200   520   720])
        sp(1)=subplot(3,1,1);
        hold on; box on
        % plotGMM(Mu(1:2,:), Sigma(1:2,1:2,:), [0.6 1.0 0.6], 1,[0.6 1.0 0.6]);
        plot(Data_for_simu(1,:),Data_for_simu(2,:),'r.')
        xlabel('$\xi_1 (mm)$','interpreter','latex','fontsize',15);
        ylabel('$\xi_2 (mm)$','interpreter','latex','fontsize',15);
        title('Simulation Results')

        sp(2)=subplot(3,1,2);
        hold on; box on
        % plotGMM(Mu([1 3],:), Sigma([1 3],[1 3],:), [0.6 1.0 0.6], 1,[0.6 1.0 0.6]);
        plot(Data_for_simu(1,:),Data_for_simu(3,:),'r.')
        xlabel('$\xi_1 (mm)$','interpreter','latex','fontsize',15);
        ylabel('$\dot{\xi}_1 (mm/s)$','interpreter','latex','fontsize',15);

        sp(3)=subplot(3,1,3);
        hold on; box on
        % plotGMM(Mu([2 4],:), Sigma([2 4],[2 4],:), [0.6 1.0 0.6], 1,[0.6 1.0 0.6]);
        plot(Data_for_simu(2,:),Data_for_simu(4,:),'r.')
        xlabel('$\xi_2 (mm)$','interpreter','latex','fontsize',15);
        ylabel('$\dot{\xi}_2 (mm/s)$','interpreter','latex','fontsize',15);

        for i=1:size(x,3)
            plot(sp(1),x(1,:,i),x(2,:,i),'linewidth',2)
            plot(sp(2),x(1,:,i),xd(1,:,i),'linewidth',2)
            plot(sp(3),x(2,:,i),xd(2,:,i),'linewidth',2)
            plot(sp(1),x(1,1,i),x(2,1,i),'ok','markersize',5,'linewidth',5)
            plot(sp(2),x(1,1,i),xd(1,1,i),'ok','markersize',5,'linewidth',5)
            plot(sp(3),x(2,1,i),xd(2,1,i),'ok','markersize',5,'linewidth',5)
        end

        for i=1:3
            axis(sp(i),'tight')
            ax=get(sp(i));
            axis(sp(i),...
                [ax.XLim(1)-(ax.XLim(2)-ax.XLim(1))/10 ax.XLim(2)+(ax.XLim(2)-ax.XLim(1))/10 ...
                ax.YLim(1)-(ax.YLim(2)-ax.YLim(1))/10 ax.YLim(2)+(ax.YLim(2)-ax.YLim(1))/10]);
            plot(sp(i),0,0,'k*','markersize',15,'linewidth',3)
            if i==1
                D = axis(sp(i));
            end
        end
        
        % plot 3D
        figure
        plot3(Data_for_simu(1,:),Data_for_simu(2,:),Data_for_simu(3,:),'r');hold on; grid on;
        for i=1:size(x,3)
            plot3(x(1,:,i),x(2,:,i),x(3,:,i),'linewidth',2);hold on; grid on;
        end
        title('3D of ground turth and SEDS')

        % plotting streamlines
        figure('name','Streamlines','position',[800   90   560   320])
        plotStreamLines(Priors,Mu,Sigma,D)
        hold on
        plot(Data_for_simu(1,:),Data_for_simu(2,:),'r.')
        plot(0,0,'k*','markersize',15,'linewidth',3)
        xlabel('$\xi_1 (mm)$','interpreter','latex','fontsize',15);
        ylabel('$\xi_2 (mm)$','interpreter','latex','fontsize',15);
        title('Streamlines of the model')
        set(gca,'position',[0.1300    0.1444    0.7750    0.7619])
    end
    
%% Plot of the data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
screen=get(0,'ScreenSize');
figure('position',[screen(:,1:2)+100 screen(:,3:4)/1.3],'name','data_GMM-demo1');
%plot 1D
for n=1:nbVar
  subplot(plot_nbVar,3,n); hold on;
  plot(demos(1,:), Data(n,:)*data_range(n)+data_mean(n), '*', 'markerSize', 1);%, 'color', [.3 .3 .3],'lineWidth', 3
  axis([min(demos(1,:)) max(demos(1,:)) min(Data(n,:)*data_range(n)+data_mean(n))-0.01 max(Data(n,:)*data_range(n)+data_mean(n))+0.01]);
  xlabel('p','fontsize',16); ylabel(['x_' num2str(n)],'fontsize',16);
end

screen=get(0,'ScreenSize');
figure('position',[screen(:,1:2)+100 screen(:,3:4)/1.3],'name','data_GMM-demo1');
%plot 1D
for n=1:nbVar
  subplot(plot_nbVar,3,n); hold on;
  plot(demos(1,:), Data_for_simu(n,:)*data_range(n)+data_mean(n), '*', 'markerSize', 1);%, 'color', [.3 .3 .3],'lineWidth', 3
%   axis([min(demos(1,:)) max(demos(1,:)) min(Data_for_simu(n,:)*data_range(n)+data_mean(n))-0.01 max(Data_for_simu(n,:)*data_range(n)+data_mean(n))+0.01]);
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
figure('position',[screen(:,1:2)+100 screen(:,3:4)/1.3],'name','GMR-demo1');
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

