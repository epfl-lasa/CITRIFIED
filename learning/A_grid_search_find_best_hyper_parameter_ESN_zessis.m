%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Rui Wu 2020.12.17
%   Grid search for ESN model for V-F-Fd data
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
pwd;
currentFolder = pwd;
if isunix
    cd /home/rui/matlab
    addpath(genpath('CITRIFIED'))
    cd CITRIFIED/
else
    cd D:\matlab2017\work
    addpath(genpath('CITRIFIED'))
    cd CITRIFIED\
end

clc;clear all; close all;
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  0 define color and path
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% data_of_exp='1018_together_with_curve_cut_1211_Z_rotate_ForceNotZero_refilter';
% data_of_exp='1018_together_with_curve_cut_1211_Z_rotate_refilter';
data_of_exp='february_learn';

%% for differend grid search
do_grid=1;

if do_grid==1
    if isunix
        %%% for linux %%%%%%%%%%%%%%%%%%%%%%%%%
        path_of_load = ['./data/data_set4learn/' data_of_exp '/'];
        path_of_save = ['./data/data_set4learn/' data_of_exp '/all_train_model/'];
        path_of_save_gridsearch = ['./data/data_set4learn/' data_of_exp '/all_train_model/'];
        path_of_plot= ['./data/figure_for_paper/' data_of_exp '/'];
    else
        path_of_load = ['.\data\data_set4learn\' data_of_exp '\'];
        path_of_save = ['.\data\data_set4learn\' data_of_exp '\all_train_model\'];
        path_of_save_gridsearch = ['.\data\data_set4learn\' data_of_exp '\all_train_model\'];
        path_of_plot= ['.\data\figure_for_paper\' data_of_exp '\'];
    end
    
    spectralRadius_0_1_kind_grid=[1:5];
    numInternalUnits_kind_gird=[1:5];
    TW_kind_gird=[1:5];

%     spectralRadius_0_1_kind_grid=[2];
%     numInternalUnits_kind_gird=[4];
%     TW_kind_gird=[4];

elseif do_grid==2 
    if isunix
        %%% for linux %%%%%%%%%%%%%%%%%%%%%%%%%
        path_of_load = ['./data/data_set4learn/' data_of_exp '/'];
        path_of_save = ['./data/data_set4learn/' data_of_exp '/all_train_model/'];
        path_of_save_gridsearch = ['./data/data_set4learn/' data_of_exp '/all_train_model/10gridsearch/'];
        path_of_plot= ['./data/figure_for_paper/' data_of_exp '/10gridsearch/'];
    else
        path_of_load = ['.\data\data_set4learn\' data_of_exp '\'];
        path_of_save = ['.\data\data_set4learn\' data_of_exp '\all_train_model\'];
        path_of_save_gridsearch = ['.\data\data_set4learn\' data_of_exp '\all_train_model\10gridsearch\'];
        path_of_plot= ['.\data\figure_for_paper\' data_of_exp '\10gridsearch\'];
    end
    
    spectralRadius_0_1_kind_grid=[1:10];
    numInternalUnits_kind_gird=[1:10];
    TW_kind_gird=[1:3];
    
%     spectralRadius_0_1_kind_best=1;
%     numInternalUnits_kind_best=9;
%     TW_kind_best=1;
end

if exist(path_of_save)==0
    status = mkdir(path_of_save); 
end
if exist(path_of_save_gridsearch)==0
    status = mkdir(path_of_save_gridsearch); 
end
if exist(path_of_plot)==0
    status = mkdir(path_of_plot); 
end

legend_time=0;timer=0;
plot_legend=[];
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  choice learning parameter
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%% choice learning_method %%%%%%%%%%%%  for differernt choice, do 10 fold CV using meanSTD to see this model is better or worse
learning_method=2;% 1 for SVM, 2 for ESN 3 using GMM see data separate
hand_choice_label=0;%1 use hand choice free motion and interaction
with_robot_data=2;% 0 only human data, 1 human with robot data, 2 only robot data

see_label=0;

k_fold=7;%% for ESN, 10 uisng 10 fold to find best model, 9 for generate hold out better test set
test_train_rate=0.4;

featuresIDs=[1;2;3;4;5;6];

for spectralRadius_0_1_kind=spectralRadius_0_1_kind_grid  % 5 5 3
    if do_grid==1
        if spectralRadius_0_1_kind==1
            spectralRadius_0_1=0.1; 
        elseif spectralRadius_0_1_kind==2
            spectralRadius_0_1=0.3;
        elseif spectralRadius_0_1_kind==3
            spectralRadius_0_1=0.5;
        elseif spectralRadius_0_1_kind==4
            spectralRadius_0_1=0.7;
        elseif spectralRadius_0_1_kind==5
            spectralRadius_0_1=1;
        end
    elseif do_grid==2
        if spectralRadius_0_1_kind==1
            spectralRadius_0_1=0.1;
        elseif spectralRadius_0_1_kind==2
            spectralRadius_0_1=0.2;
        elseif spectralRadius_0_1_kind==3
            spectralRadius_0_1=0.3;
        elseif spectralRadius_0_1_kind==4
            spectralRadius_0_1=0.4;
        elseif spectralRadius_0_1_kind==5
            spectralRadius_0_1=0.5;
        elseif spectralRadius_0_1_kind==6
            spectralRadius_0_1=0.6;
        elseif spectralRadius_0_1_kind==7
            spectralRadius_0_1=0.7;
        elseif spectralRadius_0_1_kind==8
            spectralRadius_0_1=0.8;
        elseif spectralRadius_0_1_kind==9
            spectralRadius_0_1=0.9;
        elseif spectralRadius_0_1_kind==10
            spectralRadius_0_1=1;
        end
    end
    for numInternalUnits_kind=numInternalUnits_kind_gird
        if do_grid==1
            if numInternalUnits_kind==1
                numInternalUnits=50; 
            elseif numInternalUnits_kind==2
                numInternalUnits=100;
            elseif numInternalUnits_kind==3
                numInternalUnits=250;
            elseif numInternalUnits_kind==4
                numInternalUnits=500;
            elseif numInternalUnits_kind==5
                numInternalUnits=1000;
            end
        elseif do_grid==2
            if numInternalUnits_kind==1
                numInternalUnits=10;
            elseif numInternalUnits_kind==2
                numInternalUnits=20;
            elseif numInternalUnits_kind==3
                numInternalUnits=30;
            elseif numInternalUnits_kind==4
                numInternalUnits=40;
            elseif numInternalUnits_kind==5
                numInternalUnits=50;
            elseif numInternalUnits_kind==6
                numInternalUnits=70;
            elseif numInternalUnits_kind==7
                numInternalUnits=100;
            elseif numInternalUnits_kind==8
                numInternalUnits=140;
            elseif numInternalUnits_kind==9
                numInternalUnits=200;
            elseif numInternalUnits_kind==10
                numInternalUnits=300;
            end
        end
        for TW_kind=TW_kind_gird
            clc
            if TW_kind==1
                time_of_timeWindow=0.05;%s, timeWindow is 200ms
                time_of_overlap=0.01;%s, overlap is 40ms 
            elseif TW_kind==2
                time_of_timeWindow=0.1;%s, timeWindow is 200ms
                time_of_overlap=0.02;%s, overlap is 40ms
            elseif TW_kind==3
                time_of_timeWindow=0.15;%s, timeWindow is 200ms
                time_of_overlap=0.03;%s, overlap is 40ms
            elseif TW_kind==4
                time_of_timeWindow=0.2;%s, timeWindow is 200ms
                time_of_overlap=0.04;%s, overlap is 40ms
            elseif TW_kind==5
                time_of_timeWindow=0.25;%s, timeWindow is 200ms
                time_of_overlap=0.05;%s, overlap is 40ms
            end
                
                
            %%%%%% choice phase %%%%%%%%%%%%
            % phase_choice=[1:2];
            phase_choice=[1];
            % phase_choice=[2];

            %%%%%% choice different_tissue %%%%%%%%%%%%
            if with_robot_data==1
                different_tissue=[1 2 6 7 9 10 11 12 13 14 15 16]; 
                mean_time_length=5.5300;
            elseif with_robot_data==0
                different_tissue=[1 2 6 7 9 10 11 12]; 
                mean_time_length=3.4670;

            elseif with_robot_data==2
                different_tissue=[13 14 15]; 
                mean_time_length=6.6139;
            end
            free_motion_label=1;
            Class_name={'1','2','7','9','12'};

            % different_tissue=[9:12];
            % Class_name={'9','10','11','12'};

            %%%%%% choice learning_method %%%%%%%%%%%%
            if learning_method==1
                if with_robot_data==1
                    learning_method_name=['SVM'];
                elseif with_robot_data==0
                    learning_method_name=['human_SVM'];
                elseif with_robot_data==2
                    learning_method_name=['robot_SVM'];
                end
                extFeatures=1;
                classfer=2; % 1 using radom time window separate train and test, 2 using trials as train and test

                if extFeatures==0
                % % for SVM, choice different feature to learning
                choice_feature=[4:12];%[[4:12]] not include pos and imp

                        % pdemos_pos_all_phase=train_set{ff}(1:3,:);
                        % pdemos_vel_all_phase=train_set{ff}(4:6,:);
                        % pdemos_force_all_phase=train_set{ff}(7:9,:);
                        % pdemos_f_d_all_phase=train_set{ff}(10:12,:);
                        % pdemos_imp_all_phase=train_set{ff}(13:15,:);
                        % pdemos_imp_d_all_phase=train_set{ff}(16:18,:);
                else

                end

            elseif learning_method==2
                if with_robot_data==1
                    learning_method_name=['ESN'];
                elseif with_robot_data==0
                    learning_method_name=['human_ESN'];
                elseif with_robot_data==2
                    learning_method_name=['robot_ESN'];
                end
                extFeatures=0;
                classfer=1;%  1 for linear data set; 2 for Parallel data set;


            elseif learning_method==3
                if with_robot_data==1
                    learning_method_name=['GMM'];
                elseif with_robot_data==0
                    learning_method_name=['human_GMM'];
                elseif with_robot_data==2
                    learning_method_name=['robot_GMM'];
                end
                extFeatures=1;
                choice_feature=[4:12];%[[4:12]] not include pos and imp

                        % pdemos_pos_all_phase=train_set{ff}(1:3,:);
                        % pdemos_vel_all_phase=train_set{ff}(4:6,:);
                        % pdemos_force_all_phase=train_set{ff}(7:9,:);
                        % pdemos_f_d_all_phase=train_set{ff}(10:12,:);
                        % pdemos_imp_all_phase=train_set{ff}(13:15,:);
                        % pdemos_imp_d_all_phase=train_set{ff}(16:18,:);

            end

            %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%  learning process (1): generate train and test set
            %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            label_of_data_all_phase_all_tissue=[];
            pdemos_imp_all_phase_all_tissue=[];
            pdemos_force_all_phase_all_tissue=[];
            pdemos_vel_all_phase_all_tissue=[];
            pdemos_pos_all_phase_all_tissue=[];
            pdemos_f_d_all_phase_all_tissue=[];
            pdemos_imp_d_all_phase_all_tissue=[];
            SignalFeatures_all_tissue=[];
            real_time_length_for_each_trails_cell=[];
            color_list=jet(3);

            for exp_kind=1:3
                close all;
                % clear all;
%                 clearvars -except max_phase1 max_phase2 max_phase3 cut_or_segment process_method exp_kind color_list data_of_exp path_of_load path_of_save path_of_plot;
                clc;
                if exp_kind==1
                    exp_name=['apple']
                    exp_time_good=[1:34];
                    color=color_list(exp_kind,:);
                elseif exp_kind==2
                    exp_name=['banana']
                    exp_time_good=[1:11];
                    color=color_list(exp_kind,:);
                elseif exp_kind==3
                    exp_name=['orange']
                    exp_time_good=[1:25];
                    color=color_list(exp_kind,:);
                end
                
                %% 4 load data
                if isunix
%                     filename=[path_of_load 'all_3_phase/' exp_name '_all_data_three_phase.mat'];   
                    filename=[path_of_load 'all_3_phase/' exp_name '_all_data_three_phase.mat'];   
                else
%                     filename=[path_of_load 'all_3_phase\' exp_name '_all_data_three_phase.mat'];   
                    filename=[path_of_load 'all_3_phase\' exp_name '_all_data_three_phase.mat'];   
                end
                load(filename,'time_serise_data_serial','pos_data_serial_norm_time','vel_knife_data_serial_norm_time','force_data_serial_norm_time','imp_data_serial_norm_time')

                !echo load data!

            %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %% step 1: generate data set for CV                                   %%%%%%
            %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                MSE_compare_all=[];
                %% get data mix, and use 10 fold
                verfine_time=10000;
                label_of_data_all_phase=[];
                pdemos_imp_all_phase=[];
                pdemos_force_all_phase=[];
                pdemos_vel_all_phase=[];
                pdemos_pos_all_phase=[];
                pdemos_f_d_all_phase=[];
                pdemos_imp_d_all_phase=[];
                real_time_all_phase=[];



                %% %%  get all data into train and test set
                for exp_time=exp_time_good
                    each_trail_label_of_data_all_phase=[];
                    each_trail_pdemos_imp_all_phase=[];
                    each_trail_pdemos_force_all_phase=[];
                    each_trail_pdemos_vel_all_phase=[];
                    each_trail_pdemos_pos_all_phase=[];
                    each_trail_pdemos_f_d_all_phase=[];
                    each_trail_pdemos_imp_d_all_phase=[];
                    each_trail_real_time_all_phase=[];
                    for phase=phase_choice
                        %%% mix all phase data get the data for all phases
                        label_of_data_all_phase=[label_of_data_all_phase linspace(exp_kind,exp_kind,length(time_serise_data_serial{phase,exp_time}(1,:)))];
                        real_time_all_phase=[real_time_all_phase time_serise_data_serial{phase,exp_time}(1,:)];
                        pdemos_imp_all_phase=[pdemos_imp_all_phase imp_data_serial_norm_time{phase,exp_time}([2:4],:)];
                        pdemos_force_all_phase=[pdemos_force_all_phase force_data_serial_norm_time{phase,exp_time}([2:4],:)];
                        pdemos_vel_all_phase=[pdemos_vel_all_phase vel_knife_data_serial_norm_time{phase,exp_time}([2:4],:)];
                        pdemos_pos_all_phase=[pdemos_pos_all_phase pos_data_serial_norm_time{phase,exp_time}([2:4],:)];
                        pdemos_f_d_all_phase=[pdemos_f_d_all_phase force_data_serial_norm_time{phase,exp_time}([5:7],:)];
                        pdemos_imp_d_all_phase=[pdemos_imp_d_all_phase imp_data_serial_norm_time{phase,exp_time}([5:7],:)];

                        each_trail_real_time_all_phase=[each_trail_real_time_all_phase time_serise_data_serial{phase,exp_time}(1,:)];
                        each_trail_pdemos_imp_all_phase=[each_trail_pdemos_imp_all_phase imp_data_serial_norm_time{phase,exp_time}([2:4],:)];
                        each_trail_pdemos_force_all_phase=[each_trail_pdemos_force_all_phase force_data_serial_norm_time{phase,exp_time}([2:4],:)];
                        each_trail_pdemos_vel_all_phase=[each_trail_pdemos_vel_all_phase vel_knife_data_serial_norm_time{phase,exp_time}([2:4],:)];
                        each_trail_pdemos_pos_all_phase=[each_trail_pdemos_pos_all_phase pos_data_serial_norm_time{phase,exp_time}([2:4],:)];
                        each_trail_pdemos_f_d_all_phase=[each_trail_pdemos_f_d_all_phase force_data_serial_norm_time{phase,exp_time}([5:7],:)];
                        each_trail_pdemos_imp_d_all_phase=[each_trail_pdemos_imp_d_all_phase imp_data_serial_norm_time{phase,exp_time}([5:7],:)];

                        if hand_choice_label==1 && exp_kind==14 && exp_time>=21
                            f=figure
                            plot([1:length(each_trail_pdemos_vel_all_phase(1,:))]',each_trail_pdemos_vel_all_phase(1,:));hold on;grid on;
                            plot([1:length(each_trail_pdemos_force_all_phase(1,:))]',each_trail_pdemos_force_all_phase(1,:));hold on;grid on;
                            plot([1:length(each_trail_pdemos_force_all_phase(1,:))]',each_trail_pdemos_force_all_phase(2,:));hold on;grid on;
                            plot([1:length(each_trail_pdemos_f_d_all_phase(1,:))]',each_trail_pdemos_f_d_all_phase(2,:));hold on;grid on;
                            legend('v','fc','fv','fcd','Location','best')
                            title(exp_name);
                            getXx{phase,exp_time}= cut_data_by_hand( f );
                            close(f)
                            save([path_of_save exp_name num2str(exp_time) num2str(phase) 'choice_label_by_handgetXx.mat'],'getXx');
                            !echo save choice_label_by_hand done!
                        else
                            load([path_of_save exp_name num2str(exp_time) num2str(phase) 'choice_label_by_handgetXx.mat'],'getXx');
                            !echo load choice_label_by_hand done!
                        end

                    %% merge some data set and make data is enough


                    %% based on different tissue and free motion to add label
                        if learning_method==2
                            if exp_kind==1
                                label_matrix_contact=[
                                    linspace(0,0,length(each_trail_pdemos_vel_all_phase(1,round(getXx{phase,exp_time})+1:end)));
                                    linspace(free_motion_label,free_motion_label,length(each_trail_pdemos_vel_all_phase(1,round(getXx{phase,exp_time})+1:end)));
                                    linspace(0,0,length(each_trail_pdemos_vel_all_phase(1,round(getXx{phase,exp_time})+1:end)));
                                    linspace(0,0,length(each_trail_pdemos_vel_all_phase(1,round(getXx{phase,exp_time})+1:end)));
                                    linspace(0,0,length(each_trail_pdemos_vel_all_phase(1,round(getXx{phase,exp_time})+1:end)))];
                            elseif exp_kind==2
                                label_matrix_contact=[
                                    linspace(0,0,length(each_trail_pdemos_vel_all_phase(1,round(getXx{phase,exp_time})+1:end)));
                                    linspace(0,0,length(each_trail_pdemos_vel_all_phase(1,round(getXx{phase,exp_time})+1:end)));
                                    linspace(free_motion_label,free_motion_label,length(each_trail_pdemos_vel_all_phase(1,round(getXx{phase,exp_time})+1:end)));
                                    linspace(0,0,length(each_trail_pdemos_vel_all_phase(1,round(getXx{phase,exp_time})+1:end)));
                                    linspace(0,0,length(each_trail_pdemos_vel_all_phase(1,round(getXx{phase,exp_time})+1:end)))];
                            elseif exp_kind==3
                                label_matrix_contact=[
                                    linspace(0,0,length(each_trail_pdemos_vel_all_phase(1,round(getXx{phase,exp_time})+1:end)));
                                    linspace(0,0,length(each_trail_pdemos_vel_all_phase(1,round(getXx{phase,exp_time})+1:end)));
                                    linspace(0,0,length(each_trail_pdemos_vel_all_phase(1,round(getXx{phase,exp_time})+1:end)));
                                    linspace(free_motion_label,free_motion_label,length(each_trail_pdemos_vel_all_phase(1,round(getXx{phase,exp_time})+1:end)));
                                    linspace(0,0,length(each_trail_pdemos_vel_all_phase(1,round(getXx{phase,exp_time})+1:end)))];
                            end
                            label_matrix_freemotion=[
                                linspace(free_motion_label,free_motion_label,length(each_trail_pdemos_vel_all_phase(1,1:round(getXx{phase,exp_time}))));...
                                linspace(0,0,length(each_trail_pdemos_vel_all_phase(1,1:round(getXx{phase,exp_time}))));...
                                linspace(0,0,length(each_trail_pdemos_vel_all_phase(1,1:round(getXx{phase,exp_time}))));...
                                linspace(0,0,length(each_trail_pdemos_vel_all_phase(1,1:round(getXx{phase,exp_time}))));...
                                linspace(0,0,length(each_trail_pdemos_vel_all_phase(1,1:round(getXx{phase,exp_time}))))];

                            each_trail_label_of_data_all_phase=[each_trail_label_of_data_all_phase...
                                label_matrix_freemotion ...
                                label_matrix_contact];
                        else
                            each_trail_label_of_data_all_phase=[each_trail_label_of_data_all_phase...
                            linspace(free_motion_label,free_motion_label,length(each_trail_pdemos_vel_all_phase(1,1:round(getXx{phase,exp_time}))))...
                            linspace(exp_kind_num,exp_kind_num,length(each_trail_pdemos_vel_all_phase(1,round(getXx{phase,exp_time})+1:end)))];
                        end

                    end

                %%  using slide window get feature of data
                %% cause we norm the time, so we think all approach phase is 0.8s; sample time is 
                    Signal=[each_trail_pdemos_vel_all_phase;each_trail_pdemos_force_all_phase;each_trail_pdemos_f_d_all_phase;each_trail_label_of_data_all_phase]';
                    real_time_for_each_trails=each_trail_real_time_all_phase';
                    clear each_trail_real_time_all_phase each_trail_label_of_data_all_phase each_trail_relative_time_all_phase each_trail_pdemos_imp_all_phase each_trail_pdemos_force_all_phase each_trail_pdemos_vel_all_phase each_trail_pdemos_pos_all_phase each_trail_pdemos_f_d_all_phase each_trail_pdemos_imp_d_all_phase

                    real_time_length_for_each_trails_cell=[real_time_length_for_each_trails_cell [real_time_for_each_trails(end,1)-real_time_for_each_trails(1,1);exp_kind;exp_time]];
                    if learning_method==1  || learning_method==3 %%% if svm , data just a lot of singel cell,
                        SR=(real_time_for_each_trails(end,1)-real_time_for_each_trails(1,1))/length(real_time_for_each_trails(:,1));
                    elseif learning_method==2 && classfer==1 %%% if ESN , data should be normalized by time, which mean finially we should get same number time window for each trials
                        SR=(real_time_for_each_trails(end,1)-real_time_for_each_trails(1,1))/length(real_time_for_each_trails(:,1));
                    elseif learning_method==2 && classfer==2 %%% if ESN , data should be normalized by time, which mean finially we should get same number time window for each trials
                        %%% scale and norm the time
                        scale_time=((real_time_for_each_trails(end,1)-real_time_for_each_trails(1,1))/mean_time_length);
                        real_time_for_each_trails=real_time_for_each_trails/scale_time;
                        SR=(real_time_for_each_trails(end,1)-real_time_for_each_trails(1,1))/length(real_time_for_each_trails(:,1));
                    end

                    data_num_for_time_window=time_of_timeWindow/SR;
                    data_num_for_overlap=time_of_overlap/SR;
                    l_TW=round(data_num_for_time_window);
                    delay_TW=round(data_num_for_time_window-data_num_for_overlap);

            %             lenthgTW=10;overlap=2;
            %             l_TW=lenthgTW*SR;
            %             delay_TW=l_TW-overlap*SR;

                    countTW=1;

                    if learning_method==2
                        [x y]=find(Signal(:,end-4)~=free_motion_label);
                    else
                        [x y]=find(Signal(:,end)~=free_motion_label);
                    end
                    output=[];
                    for i=1:delay_TW:length(Signal)
                        if i+l_TW>length(Signal)
                            if extFeatures
                                output{countTW}=wr_exctractFeatures(Signal(i:end,1:end-1),featuresIDs);
                                [ out1 ]=majorityvote(Signal(i:end,end)');
                                if isempty(out1)
                                    out1=free_motion_label;
                                end
                                output{countTW}=[output{countTW}(:,:) out1];
                            else
                                output{countTW}=Signal(i:end,:);
                            end
                        else
                            if extFeatures
                                output{countTW}=wr_exctractFeatures(Signal(i:i+l_TW,1:end-1),featuresIDs);
                                [ out1 ]=majorityvote(Signal(i:i+l_TW,end)');
                                if isempty(out1)
                                    out1=free_motion_label;
                                end
                                output{countTW}=[output{countTW}(:,:) out1];
                            else
                                output{countTW}=Signal(i:i+l_TW,:);
                            end
                        end
                        if round(i+l_TW)<=x(1)
                            contact_begin_here=countTW;
                        end
                        countTW=countTW+1;
                    end

                    if size(output{countTW-1},1)<5
                        output{countTW-1}=[];
                         output(cellfun(@isempty,output))=[];
                        countTW=countTW-1;
                    end

                    if learning_method==2 && classfer==2  %% for ESN to make same num of time window, using begin contact time as a gap, make time window two side
                        before_contact_TW_num=15;
                        after_contact_TW_num=5;
                        if contact_begin_here-before_contact_TW_num<0
                            exp_kind
                            exp_time
                            error('free motion time window not enough!')
                        elseif contact_begin_here+after_contact_TW_num>countTW-1
                            exp_kind
                            exp_time
                            error('contant motion time window not enough!')
                        end

                        output(contact_begin_here+after_contact_TW_num+1:end)=[];
                        output(1:contact_begin_here-before_contact_TW_num-1)=[];
            %             output1(cellfun(@isempty,output))=[];
                        countTW=size(output,2)+1;

                    end

                    each_trail_time_sequence_cell{exp_kind,exp_time}=output;

                    %%% generate feature matrix for SVM learning (no need for time sequency)
                    SignalFeatures_all_exp=[];
                    for time=1:countTW-1
                         SignalFeatures_all_exp=[SignalFeatures_all_exp; output{time}];
                    end

                    each_trail_feature{exp_kind,exp_time}=[SignalFeatures_all_exp];


                    if see_label==1
                        %%% check the free_motion_label label is right or not
                        figure
                        plot([1:length(each_trail_feature{exp_kind,exp_time}(:,1))]',each_trail_feature{exp_kind,exp_time}(:,1));hold on;grid on;
                        plot([1:length(each_trail_feature{exp_kind,exp_time}(:,1))]',each_trail_feature{exp_kind,exp_time}(:,4));hold on;grid on;
                        plot([1:length(each_trail_feature{exp_kind,exp_time}(:,1))]',each_trail_feature{exp_kind,exp_time}(:,7));hold on;grid on;
                        plot([1:length(each_trail_feature{exp_kind,exp_time}(:,1))]',each_trail_feature{exp_kind,exp_time}(:,end));hold on;grid on;

                        legend('v','f','fd','label','Location','best');
                        title('plot data time serise with label')
                    end

                end

                SignalFeatures_all_tissue=[SignalFeatures_all_tissue SignalFeatures_all_exp'];
                label_of_data_all_phase_all_tissue=[label_of_data_all_phase_all_tissue label_of_data_all_phase];
                pdemos_imp_all_phase_all_tissue=[pdemos_imp_all_phase_all_tissue pdemos_imp_all_phase];
                pdemos_force_all_phase_all_tissue=[pdemos_force_all_phase_all_tissue pdemos_force_all_phase];
                pdemos_vel_all_phase_all_tissue=[pdemos_vel_all_phase_all_tissue pdemos_vel_all_phase];
                pdemos_pos_all_phase_all_tissue=[pdemos_pos_all_phase_all_tissue pdemos_pos_all_phase];
                pdemos_f_d_all_phase_all_tissue=[pdemos_f_d_all_phase_all_tissue pdemos_f_d_all_phase];
                pdemos_imp_d_all_phase_all_tissue=[pdemos_imp_d_all_phase_all_tissue pdemos_imp_d_all_phase];

                clear label_of_data_all_phase pdemos_imp_all_phase pdemos_force_all_phase pdemos_vel_all_phase pdemos_pos_all_phase pdemos_f_d_all_phase pdemos_imp_d_all_phase


            end


            pdemos_pos_all_phase_all_tissue=[pdemos_pos_all_phase_all_tissue , zeros(3,length(pdemos_vel_all_phase_all_tissue)-length(pdemos_pos_all_phase_all_tissue))];
            pdemos_imp_all_phase_all_tissue=[pdemos_imp_all_phase_all_tissue , zeros(3,length(pdemos_vel_all_phase_all_tissue)-length(pdemos_imp_all_phase_all_tissue))];
            pdemos_imp_d_all_phase_all_tissue=[pdemos_imp_d_all_phase_all_tissue , zeros(3,length(pdemos_vel_all_phase_all_tissue)-length(pdemos_imp_d_all_phase_all_tissue))];

            if learning_method==3
                %% put all data in to MIX as follow
                MIX_data_PVFI=[pdemos_pos_all_phase_all_tissue;pdemos_vel_all_phase_all_tissue;...
                                pdemos_force_all_phase_all_tissue;pdemos_f_d_all_phase_all_tissue;...
                                pdemos_imp_all_phase_all_tissue;pdemos_imp_d_all_phase_all_tissue;...
                                label_of_data_all_phase_all_tissue];
                clear  pdemos_imp_d_all_phase pdemos_f_d_all_phase pdemos_pos_all_phase pdemos_vel_all_phase pdemos_force_all_phase pdemos_imp_all_phase label_of_data_all_phase ...
                    pdemos_imp_d_all_phasec_all_tissue pdemos_f_d_all_phase_all_tissue pdemos_pos_all_phase_all_tissue pdemos_vel_all_phase_all_tissue pdemos_force_all_phase_all_tissue pdemos_imp_all_phase_all_tissue label_of_data_all_phase_all_tissue
                targets = MIX_data_PVFI(1,:);
                k=10;
                % %%%%%%%%%%%%% 10 fold
                % CVO = cvpartition(targets,'KFold',k);
                % for ff = 1:k
                %     if (exist ('OCTAVE_VERSION', 'builtin') > 0)
                %         trIdx = training(CVO,ff);
                %         teIdx = test(CVO,ff);
                %     else
                %         trIdx = CVO.training(ff);
                %         teIdx = CVO.test(ff);
                %     end
                %     train_set{ff}=MIX_data_PVFI(:,trIdx);
                %     test_set{ff}=MIX_data_PVFI(:,teIdx);
                % end
                %%%%%%%%%%%%%% Holdout
                for ff = 1:k
                    CVO = cvpartition(length(targets),'HoldOut',0.4);
                    if (exist ('OCTAVE_VERSION', 'builtin') > 0)
                        trIdx = training(CVO,1);
                        teIdx = test(CVO,1);
                    else
                        trIdx = CVO.training(1);
                        teIdx = CVO.test(1);
                    end
                    train_set{ff}=MIX_data_PVFI(:,trIdx);
                    test_set{ff}=MIX_data_PVFI(:,teIdx);
                end
            end

            %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %% switch different learning algorithm here
            %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %% 1 is SVM
            %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%% choice using feature or orign data to train
            if learning_method==1

            %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %% 2 is ESN
            %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            elseif learning_method==2
                best_test_success=0;
                %% 1 choice different parameter
                k=k_fold;
                each_trail_feature_none_zero=each_trail_time_sequence_cell;
                each_trail_feature_none_zero(cellfun(@isempty,each_trail_feature_none_zero))=[];
                target=[1:length(each_trail_feature_none_zero)];
                if k==10
                    CVO = cvpartition(target,'KFold',k); %% k-fold used to find best model,
                else
                    CVO = cvpartition(length(target),'HoldOut',test_train_rate); %% hold out use generate better test set for E1_radom_test
                end

                for ff = 1:k
                    if (exist ('OCTAVE_VERSION', 'builtin') > 0)
                        trIdx = training(CVO,1);
                        teIdx = test(CVO,1);
                    else
                        trIdx = CVO.training(1);
                        teIdx = CVO.test(1);
                    end
                    train_set{ff}=each_trail_feature_none_zero(trIdx);
                    test_set{ff}=each_trail_feature_none_zero(teIdx);
                end


                %% 2 Holdout validation
                for ff=1:k

                    %% generate learning data set
                        train_cell=train_set{ff}';
                        test_cell=test_set{ff}';
                    %%
                    if classfer==1   %%%% using doESN_Linear_set
                        train_all_time_window=1;
                        for i=1:size(train_cell,1)
                            for j=1:size(train_cell{i},2)
                                trainesn{train_all_time_window}=train_cell{i}{j}(:,1:end-5);
                                l_tresn{train_all_time_window}=train_cell{i}{j}(:,end-4:end);
                                train_all_time_window=train_all_time_window+1;
                            end
                        end
                        test_all_time_window=1;
                        for i=1:size(test_cell,1)
                            for j=1:size(test_cell{i},2)
                                testesn{test_all_time_window}=test_cell{i}{j}(:,1:end-5);
                                l_teesn{test_all_time_window}=test_cell{i}{j}(:,end-4:end);
                                test_all_time_window=test_all_time_window+1;
                            end
                        end
                        
                        A=[spectralRadius_0_1_kind;numInternalUnits_kind;TW_kind];
                        fprintf('SR %d, NI %d, TW %d, ',A)
                        
                        [trainedEsn,Scores,errortest,errortrain,test_time,testtimestd,Con_Matrix_test,Con_Matrix_train]...
                            =doESN_Linear_set(trainesn,numInternalUnits,spectralRadius_0_1,l_tresn,testesn,l_teesn,train_all_time_window);

                        train_success(ff)=Scores{1}.score_train(2);
                        test_success(ff)=Scores{1}.score_validation(2);

                    %%
                    else classfer==2  %%% using doESN_Parallel_set
                        min_time_window_num=10000;
                        for i=1:size(train_cell,1)
                            for j=1:size(train_cell{i},2)
                                trainesn{i}{j}=train_cell{i}{j}(:,1:end-5);
                                l_tresn{i}{j}=train_cell{i}{j}(:,end-4:end);
                            end
                            if size(train_cell{i},2)<=min_time_window_num
                                min_time_window_num=size(train_cell{i},2);
                            end
                        end
                        for i=1:size(test_cell,1)
                            for j=1:size(test_cell{i},2)
                                testesn{i}{j}=test_cell{i}{j}(:,1:end-5);
                                l_teesn{i}{j}=test_cell{i}{j}(:,end-4:end);
                            end
                            if size(test_cell{i},2)<=min_time_window_num
                                min_time_window_num=size(test_cell{i},2);
                            end
                        end

                        div=min_time_window_num;
                        %% set ESN

                        [Scores,errortest,errortrain,test_time,testtimestd,Con_Matrix_test,Con_Matrix_train]...
                            =doESN_Parallel_set(trainesn,l_tresn,testesn,l_teesn,div)
                        train_success(ff)=Scores{1}.score_train(2)
                        test_success(ff)=Scores{1}.score_validation(2)
                    end

                    if test_success(ff)>best_test_success  %& train_success(ff)>=test_success(ff)
                        best_test_success=test_success(ff);
                        trainedEsn_best=trainedEsn;
                    end
                clear trainedEsn
                end
                h=figure
                plot(1:length(train_success),train_success,'b-*');hold on; grid on
                plot(1:length(test_success),test_success,'r-o');hold on; grid on
                legend('train_success','test_success')
                picture2Dsavename=[path_of_plot learning_method_name];
                saveas(h,[picture2Dsavename '.fig'])
                %% %% save the trained ESN
                
                trainedEsn=trainedEsn_best;
                estimated_generalization_success=mean(test_success);
                save([path_of_save_gridsearch learning_method_name '_TW_' num2str(TW_kind) '_InterUnit_' num2str(numInternalUnits) '_spectralRadius_' num2str(spectralRadius_0_1_kind) '_learning_4_tissue.mat'],'trainedEsn','Scores','train_cell','test_cell','estimated_generalization_success');
                !echo save ESN!

            elseif learning_method==3

            end

        end
    end
end
