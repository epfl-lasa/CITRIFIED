%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Rui Wu 2021.4.1
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
%%  0 define path
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

data_of_exp='20210330';

if isunix
    %%% for linux %%%%%%%%%%%%%%%%%%%%%%%%%
    path_of_load = ['./data/data_set4learn/' data_of_exp '/'];
    path_of_save = ['./data/data_set4learn/' data_of_exp '/'];
    path_of_plot= ['./data/figure_for_paper/' data_of_exp '/'];
else
    path_of_load = ['.\data\data_set4learn\' data_of_exp '\'];
    path_of_save = ['.\data\data_set4learn\' data_of_exp '\'];
    path_of_plot= ['.\data\figure_for_paper\' data_of_exp '\'];
end

status = mkdir(path_of_save); 
status = mkdir(path_of_plot); 
%% check the data set
if isunix
    %%% for linux %%%%%%%%%%%%%%%%%%%%%%%%%
    dirs=dir([path_of_load 'tw_data/*']);
else
    dirs=dir([path_of_load 'tw_data\*']);
end
dircell=struct2cell(dirs)';
filenames=dircell(:,1);
total_exp_time=length(filenames);
data_name_set=[];
for exp_time=1:total_exp_time-2
    name_of_exp=[filenames{exp_time+2,1}];
    C = strsplit(name_of_exp,{'_','.'});
    
    test_trail_name=[C{1} '_' C{2} '_' C{3} '_' C{4} '_' C{5} '.' C{6}];
    data=[C{1}];
    fruit=[C{2}];
    fruit_num=[C{3}];
    exp=[C{4}];
    exp_num=[C{5}];
    
    data_name_set{exp_time,1}=test_trail_name;
    data_name_set{exp_time,2}=fruit;
    data_name_set{exp_time,3}=fruit_num;
    data_name_set{exp_time,4}=exp_num;
end
clear test_trail_name fruit fruit_num exp_num

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  choice learning parameter
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%% change the tissues number %%%%%
exp_kind_total=[1:4];class_label={'apple','banana','orange','prune'};
label_num=length(exp_kind_total); % how many 0 and 1 in lable

%%%%%% color list
color_list=jet(4);
color_list(2:3,:)=[12, 247, 44;250, 157, 1;]/255;

%%%%%% train model name %%%%%%%%%%%%
learning_method_name=['robot_ESN'];

see_trial=0;

save_or_load=0;

k_fold=3;%% for ESN, 10 uisng 10 fold to find best model, 9 for generate hold out better test set
test_train_rate=0.4;

do_grid=1;
%% for differend grid search
if do_grid==1
    if isunix
        path_of_save_gridsearch = ['./data/data_set4learn/' data_of_exp '/all_train_model/10gridsearch1/'];
    else
        path_of_save_gridsearch = ['.\data\data_set4learn\' data_of_exp '\all_train_model\10gridsearch1\'];
    end
    
    spectralRadius_grid = [0.1:0.1:1]; 
    numInternalUnits_grid= [100:100:1000];
    TW_grid=[100]*0.001;
    
elseif do_grid==2 
    if isunix
        path_of_save_gridsearch = ['./data/data_set4learn/' data_of_exp '/all_train_model/10gridsearch2/'];
    else
        path_of_save_gridsearch = ['.\data\data_set4learn\' data_of_exp '\all_train_model\10gridsearch2\'];
    end
    
    spectralRadius_grid=[1:10];
    numInternalUnits_grid=[1:10];
    TW_kind_grid=[1:3];
end

if exist(path_of_save_gridsearch)==0
    status = mkdir(path_of_save_gridsearch); 
end
if exist(path_of_plot)==0
    status = mkdir(path_of_plot); 
end



%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%   get the train test set
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if save_or_load
    exp_time_for_each_fruit=1;

    for exp_time=1:total_exp_time-2
        test_trail_name=data_name_set{exp_time,1};
        fruit=data_name_set{exp_time,2};
        fruit_num=data_name_set{exp_time,3};
        exp_num=data_name_set{exp_time,4};

        close all;clc;

        %%%%% different tissue's name and color and good trials
        if strcmp(fruit,['apple'])
            exp_kind=1;
            color=color_list(exp_kind,:);
            label_code=[1 0 0 0]';
        elseif strcmp(fruit,['banana'])
            exp_kind=2;
            color=color_list(exp_kind,:);
            label_code=[0 1 0 0]';
        elseif strcmp(fruit,['orange'])
            exp_kind=3;
            color=color_list(exp_kind,:);
            label_code=[0 0 1 0]';
        elseif strcmp(fruit,['prune'])
            exp_kind=4;
            color=color_list(exp_kind,:);
            label_code=[0 0 0 1]';
        end



        %% load data
        if isunix  
            filename=[path_of_load 'tw_data/' data_of_exp '_' fruit '_' num2str(fruit_num,'%02u') '_insertion_' num2str(exp_num,'%02u') '.json'];   
        else  
            filename=[path_of_load 'tw_data/' data_of_exp '_' fruit '_' num2str(fruit_num,'%02u') '_insertion_' num2str(exp_num,'%02u') '.json'];   
        end

        %% read jason
        filterd_pose=[];filterd_vel=[];filterd_force=[];time=[];esn_input=[];esn_input_together=[];label=[];
        esn_result=1;

        file = fopen(filename);
        while ~feof(file)
            line = fgetl(file);
            message = jsondecode(line);
            % handle message data
            if ~isfield(message,'time')
    %                                 trial_name=message.metadata.trial;
            else
    %                                 time=[time message.time];
    %                                 filterd_pose=[filterd_pose message.filtered.bodies{1}.pose.position];
    %                                 filterd_vel=[filterd_vel message.filtered.bodies{1}.twist.linear];
    %                                 filterd_force=[filterd_force message.filtered.bodies{3}.wrench.force];
                if isfield(message,'esn')
                    input=[message.esn.input.time';
                        message.esn.input.depth';
                        message.esn.input.velocity_x'; 
                        message.esn.input.velocity_z'; 
                        message.esn.input.force_x'; 
                        message.esn.input.force_z'; 
                        message.esn.input.force_derivative_x'; 
                        message.esn.input.force_derivative_z'; 
                        repmat(label_code,1,length(message.esn.input.time))]';
                    if see_trial
                        esn_input_together=[esn_input_together; input];
                    end
                    esn_input{esn_result}=input(:,2:end);
                    esn_result=esn_result+1;
                end
            end

        end
        fclose(file)

        if see_trial==1
            %% plot the json data

            % figure 
            % subplot(311)
            % plot(time,filterd_pose(3,:));legend('pose')
            % subplot(312)
            % plot(time,filterd_vel(3,:));legend('vel')
            % subplot(313)
            % plot(time,filterd_force(3,:));legend('force')

            figure
            subplot(411)
            plot(esn_input_together(1,:),esn_input_together(2,:));legend('pose')
            subplot(412)
            plot(esn_input_together(1,:),esn_input_together(4,:));legend('vel_z')
            subplot(413)
            plot(esn_input_together(1,:),esn_input_together(6,:));legend('force_z')
            subplot(414)
            plot(esn_input_together(1,:),esn_input_together(8,:));legend('force_d_z')

            figure
            subplot(411)
            plot(esn_input_together(1,:),esn_input_together(2,:));legend('pose')
            subplot(412)
            plot(esn_input_together(1,:),esn_input_together(3,:));legend('vel_y')
            subplot(413)
            plot(esn_input_together(1,:),esn_input_together(5,:));legend('force_y')
            subplot(414)
            plot(esn_input_together(1,:),esn_input_together(7,:));legend('force_d_y')
        end

        if exp_time_for_each_fruit<=31
            each_trail_time_sequence_cell{exp_kind,exp_time_for_each_fruit}=esn_input;
            exp_time_for_each_fruit=exp_time_for_each_fruit+1;
        else
            exp_time_for_each_fruit=1;
            each_trail_time_sequence_cell{exp_kind,exp_time_for_each_fruit}=esn_input;
        end

        clear label_of_data_all_phase pdemos_imp_all_phase pdemos_force_all_phase pdemos_vel_all_phase pdemos_pos_all_phase pdemos_f_d_all_phase pdemos_imp_d_all_phase

    end
    save([path_of_save 'all_data_in_mat.mat'],'each_trail_time_sequence_cell');
    !echo save all data in mat!
else
    load([path_of_save 'all_data_in_mat.mat'],'each_trail_time_sequence_cell');
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% run grid search
    for spectralRadius_0_1=spectralRadius_grid 
        for numInternalUnits=numInternalUnits_grid
            for TW=TW_grid

                time_of_timeWindow=TW;%s, timeWindow is 200ms
                time_of_overlap=TW/5;%s, overlap is 40ms 



                %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %% run ESN
                %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                best_test_success=0;
                %% 1 choice different parameter
                k=k_fold;
                each_trail_feature_none_zero=each_trail_time_sequence_cell;
                each_trail_feature_none_zero(cellfun(@isempty,each_trail_feature_none_zero))=[];
                target=[1:length(each_trail_feature_none_zero)];

                for ff = 1:k
                    CVO = cvpartition(length(target),'HoldOut',test_train_rate); %% hold out use generate better test set for E1_radom_test
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

                    A=[spectralRadius_0_1;numInternalUnits;TW];
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

    %             h=figure
    %             plot(1:length(train_success),train_success,'b-*');hold on; grid on
    %             plot(1:length(test_success),test_success,'r-o');hold on; grid on
    %             legend('train_success','test_success')
    %             picture2Dsavename=[path_of_plot learning_method_name];
    %             saveas(h,[picture2Dsavename '.fig'])
                %% %% save the trained ESN

                trainedEsn=trainedEsn_best;
                estimated_generalization_success=mean(test_success);
                save([path_of_save_gridsearch learning_method_name '_TW_' num2str(TW) '_InterUnit_' num2str(numInternalUnits) '_spectralRadius_' num2str(spectralRadius_0_1) '_learning_4_tissue.mat'],'trainedEsn','Scores','train_cell','test_cell','estimated_generalization_success');
                !echo save ESN!

                clear trainedEsn trainedEsn_best

            end
        end
    end
end