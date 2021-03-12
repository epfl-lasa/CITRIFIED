%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Rui Wu 2020.12.17
%   Grid search for ESN model for V-F-Fd data
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
pwd;
currentFolder = pwd;
if isunix
    cd /home/wr
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

% data_of_exp='february_learn';
data_of_exp='february_new_learn';

%% for differend grid search
do_grid=1;

if do_grid==1
    if isunix
        %%% for linux %%%%%%%%%%%%%%%%%%%%%%%%%
        path_of_load = ['./data/data_set4learn/' data_of_exp '/'];
        path_of_cutpoint = ['./data/data_set4learn/' data_of_exp '/cut_point/'];
        path_of_save_gridsearch = ['./data/data_set4learn/' data_of_exp '/all_train_model/10gridsearch1/'];
        path_of_plot= ['./data/figure_for_paper/' data_of_exp '/'];
    else
        path_of_load = ['.\data\data_set4learn\' data_of_exp '\'];
        path_of_cutpoint = ['.\data\data_set4learn\' data_of_exp '\cut_point\'];
        path_of_save_gridsearch = ['.\data\data_set4learn\' data_of_exp '\all_train_model\10gridsearch1\'];
        path_of_plot= ['.\data\figure_for_paper\' data_of_exp '\'];
    end
    
    spectralRadius_0_1_kind_grid=[1:10];
    numInternalUnits_kind_grid=[1:10];
    TW_kind_grid=[1:5];
    
    spectralRadius = [0.1:0.1:1]; 
    numInternalUnits1= [100:100:1000];
    TW_kind_1=[50:50:250]*0.001;
    overlap1=[10:10:50]*0.001;

elseif do_grid==2 
    if isunix
        %%% for linux %%%%%%%%%%%%%%%%%%%%%%%%%
        path_of_load = ['./data/data_set4learn/' data_of_exp '/'];
        path_of_cutpoint = ['./data/data_set4learn/' data_of_exp '/cut_point/'];
        path_of_save_gridsearch = ['./data/data_set4learn/' data_of_exp '/all_train_model/10gridsearch2/'];
        path_of_plot= ['./data/figure_for_paper/' data_of_exp '/10gridsearch/'];
    else
        path_of_load = ['.\data\data_set4learn\' data_of_exp '\'];
        path_of_cutpoint = ['.\data\data_set4learn\' data_of_exp '\cut_point\'];
        path_of_save_gridsearch = ['.\data\data_set4learn\' data_of_exp '\all_train_model\10gridsearch2\'];
        path_of_plot= ['.\data\figure_for_paper\' data_of_exp '\10gridsearch\'];
    end
    
    spectralRadius_0_1_kind_grid=[1:10];
    numInternalUnits_kind_grid=[1:10];
    TW_kind_grid=[1:3];
    
end

if exist(path_of_cutpoint)==0
    status = mkdir(path_of_cutpoint); 
end
if exist(path_of_save_gridsearch)==0
    status = mkdir(path_of_save_gridsearch); 
end
if exist(path_of_plot)==0
    status = mkdir(path_of_plot); 
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  choice learning parameter
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%% choice phase %%%%%%%%%%%%
phase_choice=[2:3];

%%%%% change the tissues number %%%%%
exp_kind_total=[1:3];
label_num=3;

k_fold=9;%% for ESN, 10 uisng 10 fold to find best model, 9 for generate hold out better test set
test_train_rate=0.4;

%%%%%% train model name %%%%%%%%%%%%
learning_method_name=['robot_ESN'];

see_trial=0;

%% run grid search
for spectralRadius_0_1_kind=spectralRadius_0_1_kind_grid  % 5 5 3

    spectralRadius_0_1=spectralRadius(spectralRadius_0_1_kind);
    
    for numInternalUnits_kind=numInternalUnits_kind_grid
        
        numInternalUnits=numInternalUnits1(numInternalUnits_kind);
        
        for TW_kind=TW_kind_grid
            
            time_of_timeWindow=TW_kind_1(TW_kind);%s, timeWindow is 200ms
            time_of_overlap=overlap1(TW_kind);%s, overlap is 40ms 

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
            color_list=jet(max(exp_kind_total));

            for exp_kind=exp_kind_total
                close all;
                % clear all;
%                 clearvars -except max_phase1 max_phase2 max_phase3 cut_or_segment process_method exp_kind color_list data_of_exp path_of_load path_of_save path_of_plot;
                clc;
                
                %%%%% different tissue's name and color and good trials
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
                    exp_time_good=[1:23];
                    color=color_list(exp_kind,:);
                end
                
                %% load data
                if isunix  
                    filename=[path_of_load 'all_3_phase/' exp_name '_all_data_three_phase.mat'];   
                else  
                    filename=[path_of_load 'all_3_phase\' exp_name '_all_data_three_phase.mat'];   
                end
                load(filename,'time_serise_data_serial','pos_data_serial_norm_time','vel_knife_data_serial_norm_time','force_data_serial_norm_time','imp_data_serial_norm_time')

                !echo load data!

            %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %% step 1: generate data set for CV                                   %%%%%%
            %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
                    label_of_data_all_phase=[];
                    pdemos_imp_all_phase=[];
                    pdemos_force_all_phase=[];
                    pdemos_vel_all_phase=[];
                    pdemos_pos_all_phase=[];
                    pdemos_f_d_all_phase=[];
                    pdemos_imp_d_all_phase=[];
                    real_time_all_phase=[];
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
 
                    end
                    
                    %%% add label in dataset
                    if exp_kind==1
                        label_matrix_contact=[
                            linspace(1,1,length(each_trail_pdemos_vel_all_phase(1,:)));
                            linspace(0,0,length(each_trail_pdemos_vel_all_phase(1,:)));
                            linspace(0,0,length(each_trail_pdemos_vel_all_phase(1,:)))];
                    elseif exp_kind==2
                        label_matrix_contact=[
                            linspace(0,0,length(each_trail_pdemos_vel_all_phase(1,:)));
                            linspace(1,1,length(each_trail_pdemos_vel_all_phase(1,:)));
                            linspace(0,0,length(each_trail_pdemos_vel_all_phase(1,:)))];
                    elseif exp_kind==3
                        label_matrix_contact=[
                            linspace(0,0,length(each_trail_pdemos_vel_all_phase(1,:)));
                            linspace(0,0,length(each_trail_pdemos_vel_all_phase(1,:)));
                            linspace(1,1,length(each_trail_pdemos_vel_all_phase(1,:)))];
                    end

                    each_trail_label_of_data_all_phase=[each_trail_label_of_data_all_phase...
                            label_matrix_contact];


                    %%  using slide window get feature of data
                    Signal=[-(each_trail_pdemos_pos_all_phase(3,:)-each_trail_pdemos_pos_all_phase(3,1));each_trail_pdemos_vel_all_phase(3,:);...
                        each_trail_pdemos_force_all_phase(3,:);each_trail_pdemos_f_d_all_phase(3,:);...
                        each_trail_label_of_data_all_phase]';
                    
                    real_time_for_each_trails=each_trail_real_time_all_phase';
                    clear each_trail_real_time_all_phase each_trail_label_of_data_all_phase each_trail_relative_time_all_phase each_trail_pdemos_imp_all_phase each_trail_pdemos_force_all_phase each_trail_pdemos_vel_all_phase each_trail_pdemos_pos_all_phase each_trail_pdemos_f_d_all_phase each_trail_pdemos_imp_d_all_phase

                    real_time_length_for_each_trails_cell=[real_time_length_for_each_trails_cell [real_time_for_each_trails(end,1)-real_time_for_each_trails(1,1);exp_kind;exp_time]];
                    SR=(real_time_for_each_trails(end,1)-real_time_for_each_trails(1,1))/length(real_time_for_each_trails(:,1));

                    data_num_for_time_window=time_of_timeWindow/SR;
                    data_num_for_overlap=time_of_overlap/SR;
                    l_TW=round(data_num_for_time_window);
                    delay_TW=round(data_num_for_time_window-data_num_for_overlap);

                    countTW=1;

                    [x y]=find(Signal(:,end-length(exp_kind_total))~=1);

                    output=[];
                    for i=1:delay_TW:length(Signal)
                        if i+l_TW>length(Signal)
                            output{countTW}=Signal(i:end,:);
                        else
                            output{countTW}=Signal(i:i+l_TW,:);
                        end
                        
                        if round(i+l_TW)<=x(1)
                            contact_begin_here=countTW;
                        end
                        countTW=countTW+1;
                    end
                    
                    %%% delet last tw if data too short
                    if size(output{countTW-1},1)<5
                        output{countTW-1}=[];
                         output(cellfun(@isempty,output))=[];
                        countTW=countTW-1;
                    end

%                     if learning_method==2 && classfer==2  %% for ESN to make same num of time window, using begin contact time as a gap, make time window two side
%                         before_contact_TW_num=15;
%                         after_contact_TW_num=5;
%                         if contact_begin_here-before_contact_TW_num<0
%                             exp_kind
%                             exp_time
%                             error('free motion time window not enough!')
%                         elseif contact_begin_here+after_contact_TW_num>countTW-1
%                             exp_kind
%                             exp_time
%                             error('contant motion time window not enough!')
%                         end
% 
%                         output(contact_begin_here+after_contact_TW_num+1:end)=[];
%                         output(1:contact_begin_here-before_contact_TW_num-1)=[];
%             %             output1(cellfun(@isempty,output))=[];
%                         countTW=size(output,2)+1;
% 
%                     end

                    each_trail_time_sequence_cell{exp_kind,exp_time}=output;

                    %%% generate feature matrix for SVM learning (no need for time sequency)
                    SignalFeatures_all_exp=[];
                    for time=1:countTW-1
                         SignalFeatures_all_exp=[SignalFeatures_all_exp; output{time}];
                    end

                    each_trail_feature{exp_kind,exp_time}=[SignalFeatures_all_exp];

                    if see_trial==1
                        %%% check the 1 label is right or not
                        figure
                        subplot(221)
                        plot([1:length(each_trail_feature{exp_kind,exp_time}(:,1))]',each_trail_feature{exp_kind,exp_time}(:,1));hold on;grid on;
                        legend('depth','Location','best');
                        subplot(222)
                        plot([1:length(each_trail_feature{exp_kind,exp_time}(:,1))]',each_trail_feature{exp_kind,exp_time}(:,2));hold on;grid on;
                        legend('v','Location','best');
                        subplot(223)
                        plot([1:length(each_trail_feature{exp_kind,exp_time}(:,1))]',each_trail_feature{exp_kind,exp_time}(:,3));hold on;grid on;
                        legend('f','Location','best');
                        subplot(224)
                        plot([1:length(each_trail_feature{exp_kind,exp_time}(:,1))]',each_trail_feature{exp_kind,exp_time}(:,4));hold on;grid on;
                        legend('fd','Location','best');
                        suptitle('plot data time serise with label')
                        
                        figure
                        subplot(221)
                        plot([1:length(Signal(:,1))]',Signal(:,1));hold on;grid on;
                        legend('depth','Location','best');
                        subplot(222)
                        plot([1:length(Signal(:,1))]',Signal(:,2));hold on;grid on;
                        legend('v','Location','best');
                        subplot(223)
                        plot([1:length(Signal(:,1))]',Signal(:,3));hold on;grid on;
                        legend('f','Location','best');
                        subplot(224)
                        plot([1:length(Signal(:,1))]',Signal(:,4));hold on;grid on;
                        legend('fd','Location','best');
                        suptitle('plot data time serise with label')
                    end

                end

%                 SignalFeatures_all_tissue=[SignalFeatures_all_tissue SignalFeatures_all_exp'];
%                 label_of_data_all_phase_all_tissue=[label_of_data_all_phase_all_tissue label_of_data_all_phase];
%                 pdemos_imp_all_phase_all_tissue=[pdemos_imp_all_phase_all_tissue pdemos_imp_all_phase];
%                 pdemos_force_all_phase_all_tissue=[pdemos_force_all_phase_all_tissue pdemos_force_all_phase];
%                 pdemos_vel_all_phase_all_tissue=[pdemos_vel_all_phase_all_tissue pdemos_vel_all_phase];
%                 pdemos_pos_all_phase_all_tissue=[pdemos_pos_all_phase_all_tissue pdemos_pos_all_phase];
%                 pdemos_f_d_all_phase_all_tissue=[pdemos_f_d_all_phase_all_tissue pdemos_f_d_all_phase];
%                 pdemos_imp_d_all_phase_all_tissue=[pdemos_imp_d_all_phase_all_tissue pdemos_imp_d_all_phase];

                clear label_of_data_all_phase pdemos_imp_all_phase pdemos_force_all_phase pdemos_vel_all_phase pdemos_pos_all_phase pdemos_f_d_all_phase pdemos_imp_d_all_phase


            end


%             pdemos_pos_all_phase_all_tissue=[pdemos_pos_all_phase_all_tissue , zeros(3,length(pdemos_vel_all_phase_all_tissue)-length(pdemos_pos_all_phase_all_tissue))];
%             pdemos_imp_all_phase_all_tissue=[pdemos_imp_all_phase_all_tissue , zeros(3,length(pdemos_vel_all_phase_all_tissue)-length(pdemos_imp_all_phase_all_tissue))];
%             pdemos_imp_d_all_phase_all_tissue=[pdemos_imp_d_all_phase_all_tissue , zeros(3,length(pdemos_vel_all_phase_all_tissue)-length(pdemos_imp_d_all_phase_all_tissue))];


            
            %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %% run ESN
            %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
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

                %% using doESN_Linear_set
                train_all_time_window=1;
                for i=1:size(train_cell,1)
                    for j=1:size(train_cell{i},2)
                        trainesn{train_all_time_window}=train_cell{i}{j}(:,1:end-label_num);
                        l_tresn{train_all_time_window}=train_cell{i}{j}(:,end-label_num+1:end);
                        train_all_time_window=train_all_time_window+1;
                    end
                end
                test_all_time_window=1;
                for i=1:size(test_cell,1)
                    for j=1:size(test_cell{i},2)
                        testesn{test_all_time_window}=test_cell{i}{j}(:,1:end-label_num);
                        l_teesn{test_all_time_window}=test_cell{i}{j}(:,end-label_num+1:end);
                        test_all_time_window=test_all_time_window+1;
                    end
                end

                A=[spectralRadius_0_1_kind;numInternalUnits_kind;TW_kind];
                fprintf('SR %d, NI %d, TW %d, ',A)

                [trainedEsn,Scores,errortest,errortrain,test_time,testtimestd,Con_Matrix_test,Con_Matrix_train]...
                    =doESN_Linear_set(trainesn,numInternalUnits,spectralRadius_0_1,l_tresn,testesn,l_teesn,train_all_time_window);

                train_success(ff)=Scores{1}.score_train(2);
                test_success(ff)=Scores{1}.score_validation(2);

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

            clear trainedEsn trainedEsn_best


        end
    end
end
