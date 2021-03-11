%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Rui Wu 2020.12.17
%   show best result in grid search and plot all grid search result and
%   show best
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
%%  0 define learning model path
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% data_of_exp='1018_together_with_curve_cut_1211_Z_rotate_ForceNotZero_refilter';
% data_of_exp='1018_together_with_curve_cut_1211_Z_rotate_refilter';
data_of_exp='february_learn';

%% for differend grid search
do_grid=1;

if do_grid==1
    if isunix
        %%% for linux %%%%%%%%%%%%%%%%%%%%%%%%%
        path_of_load_model = ['./data/data_set4learn/' data_of_exp '/all_train_model/'];
        path_of_plot= ['./data/figure_for_paper/' data_of_exp '/'];
    else
        path_of_load_model = ['.\data\data_set4learn\' data_of_exp '\all_train_model\'];
        path_of_plot= ['.\data\figure_for_paper\' data_of_exp '\'];
    end
    spectralRadius_0_1_kind_grid=[1:5];
    numInternalUnits_kind_gird=[1:5];
    TW_kind_gird=[1:5];
    
    spectralRadius_0_1_kind_best=spectralRadius_0_1_kind_grid;
    numInternalUnits_kind_best=numInternalUnits_kind_gird;
    TW_kind_best=TW_kind_gird;
    
%     spectralRadius_0_1_kind_best=2;
%     numInternalUnits_kind_best=4;
%     TW_kind_best=4;
    

elseif do_grid==2 
    if isunix
        %%% for linux %%%%%%%%%%%%%%%%%%%%%%%%%
        path_of_load_model = ['./data/data_set4learn/' data_of_exp '/all_train_model/10gridsearch/'];
        path_of_plot= ['./data/figure_for_paper/' data_of_exp '/10gridsearch/'];
    else
        path_of_load_model = ['.\data\data_set4learn\' data_of_exp '\all_train_model\10gridsearch\'];
        path_of_plot= ['.\data\figure_for_paper\' data_of_exp '\10gridsearch\'];
    end
    spectralRadius_0_1_kind_grid=[1:10];
    numInternalUnits_kind_gird=[1:10];
    TW_kind_gird=[1:3];
    spectralRadius_0_1_kind_best=1;
    numInternalUnits_kind_best=9;
    TW_kind_best=1;
end

legend_time=0;timer=0;
plot_legend=[];

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  choice learning parameter
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%% choice learning_method %%%%%%%%%%%%
learning_method=2;% 1 for SVM, 2 for ESN
%%%%%% choice learning_method model %%%%%%%%%%%%
with_robot_data=2;% 0 model only human data, 1 model human with robot data, 2  model only robot data
% TW_kind=3;%3 is best 150ms
less_feature_no_vel=0;


if learning_method==1
    if with_robot_data==1
        learning_method_name=['SVM'];
    elseif with_robot_data==0
        learning_method_name=['human_SVM'];
    elseif with_robot_data==2
        learning_method_name=['robot_SVM'];
    end
    extFeatures=1; 
    classfer=2;
    k_fold=10;test_train_rate=0.2;
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
    classfer=2;
    k_fold=10;test_train_rate=0.2;
    
elseif learning_method==3
    if with_robot_data==1
        learning_method_name=['GMM'];
    elseif with_robot_data==0
        learning_method_name=['human_GMM'];
    elseif with_robot_data==2
        learning_method_name=['robot_GMM'];
    end
    % % for GMM
    KMM_matrix=[1];% for GMM-based learning method, change this;
                    %for SVM just set it as 1
end


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% switch grid search
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% load learning model

%     fid = fopen([path_of_load_model 'grid_search_result.txt'],'wt');


for spectralRadius_0_1_kind=spectralRadius_0_1_kind_best  % 5 5 3
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
    for numInternalUnits_kind=numInternalUnits_kind_best
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
        for TW_kind=TW_kind_best
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
            %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %% switch different learning algorithm here
            %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %% load learning model
            if less_feature_no_vel==0
                load([path_of_load_model learning_method_name '_TW_' num2str(TW_kind) '_InterUnit_' num2str(numInternalUnits) '_spectralRadius_' num2str(spectralRadius_0_1_kind) '_learning_4_tissue.mat']);
            else
                load([path_of_load_model learning_method_name '_TW_' num2str(TW_kind) '_InterUnit_' num2str(numInternalUnits) '_spectralRadius_' num2str(spectralRadius_0_1_kind) '_less_feature_no_vel.mat']);
            end

            if with_robot_data==0 %%using human data train and using robot data test
                if learning_method==1
                    learning_method_name=['robot_SVM'];
                else learning_method==2
                    learning_method_name=['robot_ESN'];
                end
                if less_feature_no_vel==0
                    load([path_of_load_model learning_method_name '_TW_' num2str(TW_kind) '_learning_4_tissue.mat'],'train_cell','test_cell');
                else
                    load([path_of_load_model learning_method_name '_TW_' num2str(TW_kind) '_less_feature_no_vel.mat'],'train_cell','test_cell');
                end
            %     test_cell=[test_cell; train_cell]
            end

            %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %% MV to get result
            %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            success_time=0;
            trial_A=0;trial_B=0;trial_C=0;trial_D=0;
            success_time_A=0;success_time_B=0;success_time_C=0;success_time_D=0;
            predictA_to_A=0;predictA_to_B=0;predictA_to_C=0;predictA_to_D=0;
            predictB_to_A=0;predictB_to_B=0;predictB_to_C=0;predictB_to_D=0;
            predictC_to_A=0;predictC_to_B=0;predictC_to_C=0;predictC_to_D=0;
            predictD_to_A=0;predictD_to_B=0;predictD_to_C=0;predictD_to_D=0;
            score_validation=[];
            fail_trials=[];

            for trials_num=1:size(test_cell,1)
                clearvars -except numInternalUnits_kind_best TW_kind_best do_grid fid learning_method_name extFeatures classfer k_fold test_train_rate with_robot_data spectralRadius_0_1 numInternalUnits time_of_timeWindow time_of_overlap learning_method_name path_of_load_model spectralRadius_0_1_kind numInternalUnits_kind TW_kind less_feature_no_vel testtimestd test_time SVMModel learning_method fail_trials TW_success_for_each_trial success_time_A trial_A success_time_B trial_B success_time_C trial_C...
                    success_time_D  trial_D test_cell nForgetPoints nOutputUnits score_validation trials_num trainedEsn success_rate_test_for_each_trial...
                    predict*

                %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %% 1 is SVM
                %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                if learning_method==1

                %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %% 2 is ESN
                %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                elseif learning_method==2
                    disp('Testing ESN ............');
                    nForgetPoints=0;
                    nOutputUnits=5;

                    for TW_num=1:size(test_cell{trials_num},2)
                        testInputSequence{TW_num}=test_cell{trials_num}{TW_num}(:,1:end-5);
                        testOutputSequence{TW_num}=test_cell{trials_num}{TW_num}(:,end-4:end);
                    end

                    calc_time=zeros(length(testInputSequence),1);

                    predictedTestOutput = [];
                    for TW_num=1:length(testInputSequence)
                        predictedTestOutput{TW_num} = zeros(length(testInputSequence{TW_num})-nForgetPoints, 5);
                        tic
                        predictedTestOutput{TW_num} = test_esn(testInputSequence{TW_num},  trainedEsn, nForgetPoints) ; 
                        calc_time(TW_num)=toc;
                    end

                    test_time{trials_num}=sum(calc_time)/length(testInputSequence);
                    testtimestd{trials_num}=std(calc_time);

                    [predicted_class_time_squence,all_output_test, av_predicteTestdOutput, success_rate_test, av_confidence_all_test, std_confidence_all_test, av_max_conf_test, std_max_conf_test, errortest{trials_num},Con_Matrix_test{trials_num}]...
                    = S_classify2_WR_for_real_test(predictedTestOutput, 3, trials_num, 'test',nOutputUnits);

                    %%% because S_classify2_WR_for_real_test only have 98% success rate
                    %%% change funciton
            %         [real_class_time_squence,all_output_test, av_predicteTestdOutput, success_rate_test, av_confidence_all_test, std_confidence_all_test, av_max_conf_test, std_max_conf_test, errortest{trials_num},Con_Matrix_test{trials_num}]...
            %         = S_classify2_WR_for_real_test(testOutputSequence, 3, trials_num, 'test',nOutputUnits);
                    for TW_num=1:length(testOutputSequence)

                        TW_mean=mean(testOutputSequence{TW_num},1);
                        [val, ind] = max(TW_mean./sum(TW_mean));
                        predicted_class = zeros(1,nOutputUnits);
                        predicted_class(ind) = 1;

                        real_class_time_squence{TW_num}=predicted_class;
                    end

                    [all_output_test, av_predicteTestdOutput, success_rate_test, av_confidence_all_test, std_confidence_all_test, av_max_conf_test, std_max_conf_test, errortest{trials_num},Con_Matrix_test{trials_num}]...
                        = S_classify2_WR(predictedTestOutput, testOutputSequence, 3, trials_num, 'test',nOutputUnits);

                    score_validation = [score_validation; trials_num success_rate_test av_confidence_all_test std_confidence_all_test av_max_conf_test std_max_conf_test];

                    for TW_num=1:length(predicted_class_time_squence)
                        if predicted_class_time_squence{TW_num}==[1 0 0 0 0]
                            label(TW_num)=1;
                        elseif predicted_class_time_squence{TW_num}==[0 1 0 0 0]
                            label(TW_num)=2;
                        elseif predicted_class_time_squence{TW_num}==[0 0 1 0 0]
                            label(TW_num)=7;
                        elseif predicted_class_time_squence{TW_num}==[0 0 0 1 0]
                            label(TW_num)=9;
                        elseif predicted_class_time_squence{TW_num}==[0 0 0 0 1]
                            label(TW_num)=12;
                        end

                        if real_class_time_squence{TW_num}==[1 0 0 0 0]
                            real_label(TW_num)=1;
                        elseif real_class_time_squence{TW_num}==[0 1 0 0 0]
                            real_label(TW_num)=2;
                        elseif real_class_time_squence{TW_num}==[0 0 1 0 0]
                            real_label(TW_num)=7;
                        elseif real_class_time_squence{TW_num}==[0 0 0 1 0]
                            real_label(TW_num)=9;
                        elseif real_class_time_squence{TW_num}==[0 0 0 0 1]
                            real_label(TW_num)=12;
                        end
                    end

                end

                %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %% using MV to decide which tissue it is
                %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                each_TW_success=0;
                 for TW_num=1:length(label)
                    each_TW_success=each_TW_success+isequal(label(TW_num),real_label(TW_num));
                 end

                 TW_success_for_each_trial(trials_num)=(each_TW_success/length(label))*100;%% these two should be similar
            %      success_rate_test_for_each_trial(trials_num)=success_rate_test;

                for TW_num=5:length(label)
                    label_perdict=[label(TW_num-4) label(TW_num-3) label(TW_num-2) label(TW_num-1) label(TW_num)];
                    [out] = majorityvote(label_perdict);
                    out1(TW_num-4)=out;

                    real_label_perdict=[real_label(TW_num-4) real_label(TW_num-3) real_label(TW_num-2) real_label(TW_num-1) real_label(TW_num)];
                    [real_out] = majorityvote(real_label_perdict );
                    real_out1(TW_num-4)=real_out;
                end



                id = real_out1(:)==1;
                real_out1(id) = [];
                label_real_finial=majorityvote(real_out1 );

                id = out1(:)==1;
                out1(id) = [];
                label_predict_finial=majorityvote(out1 );

                 %%%% plot result
                 plot_input_time_serise=[];
                 for TW_num=1:size(test_cell{trials_num},2)
                        testInputSequence{TW_num}=test_cell{trials_num}{TW_num}(:,1:end-5);
                        testOutputSequence{TW_num}=test_cell{trials_num}{TW_num}(:,end-4:end);
                        plot_input_TW(TW_num,:)=mean(testInputSequence{TW_num});
                        plot_input_time_serise=[plot_input_time_serise; testInputSequence{TW_num}];
                 end
                time_for_plot=((1:length(plot_input_time_serise(:,1)))/length(plot_input_time_serise(:,1)))*length(real_label);

%                 figure
%                 plot(time_for_plot,plot_input_time_serise(:,2)*80);hold on;
%                 plot(time_for_plot,plot_input_time_serise(:,5)*4);hold on;
%                 plot(time_for_plot,plot_input_time_serise(:,8)*1);hold on;
% 
%                 plot(1:length(real_label),real_label(:),'-*');hold on;
%                 plot(1:length(label),label(:),'-o');hold on;
% %                 plot([1:length(out1)],out1(:),'-o');hold on;
% %                 plot([1:length(real_out1)],real_out1(:),'-+');hold on;
%                 legend('velocity','force','force deritive','real-label','predict-label','Location','best');%,'MV-label','MV-real-out'
%             %     legend('real-label','label','Location','best');
%                 title(['class real label ' num2str(label_real_finial) ' as ' num2str(label_predict_finial)]);

                fail_exp=[];
                if label_predict_finial==label_real_finial
                    if label_real_finial==2
                        success_time_A=success_time_A+1;
                    elseif label_real_finial==7
                        success_time_B=success_time_B+1;
                    elseif label_real_finial==9
                        success_time_C=success_time_C+1;
                    elseif label_real_finial==12
                        success_time_D=success_time_D+1;
                    end
                    fail_exp=[];
                elseif label_predict_finial==2
                    fail_exp=trials_num;
                    if label_real_finial==2
                        success_time_A=success_time_A+1;
                    elseif label_real_finial==7
                        predictB_to_A=predictB_to_A+1;
                    elseif label_real_finial==9
                        predictC_to_A=predictC_to_A+1;
                    elseif label_real_finial==12
                        predictD_to_A=predictD_to_A+1;
                    end
                elseif label_predict_finial==7
                    fail_exp=trials_num;
                    if label_real_finial==2
                        predictA_to_B=predictA_to_B+1;
                    elseif label_real_finial==7
                        success_time_B=success_time_B+1;
                    elseif label_real_finial==9
                        predictC_to_B=predictC_to_B+1;
                    elseif label_real_finial==12
                        predictD_to_B=predictD_to_B+1;
                    end
                elseif label_predict_finial==9
                    fail_exp=trials_num;
                    if label_real_finial==2
                        predictA_to_C=predictA_to_C+1;
                    elseif label_real_finial==7
                        predictB_to_C=predictB_to_C+1;
                    elseif label_real_finial==9
                        predictC_to_C=predictC_to_C+1;
                    elseif label_real_finial==12
                        predictD_to_C=predictD_to_C+1;
                    end
                elseif label_predict_finial==12
                    fail_exp=trials_num;
                    if label_real_finial==2
                        predictA_to_D=predictA_to_D+1;
                    elseif label_real_finial==7
                        predictB_to_D=predictB_to_D+1;
                    elseif label_real_finial==9
                        predictC_to_D=predictC_to_D+1;
                    elseif label_real_finial==12
                        predictD_to_D=predictD_to_D+1;
                    end
                end

                if label_real_finial==2
                    trial_A=trial_A+1;
                elseif label_real_finial==7
                    trial_B=trial_B+1;
                elseif label_real_finial==9
                    trial_C=trial_C+1;
                elseif label_real_finial==12
                    trial_D=trial_D+1;
                end

                fail_trials{trials_num}=[fail_exp label_real_finial label_predict_finial];



            end

            success_rate{2}=[success_time_A/trial_A];
            success_rate{7}=[success_time_B/trial_B];
            success_rate{9}=[success_time_C/trial_C];
%             success_rate{12}=[success_time_D/trial_D];

            success_std{2}=sqrt(success_time_A*(1-(success_time_A/trial_A))^2/trial_A);
            success_std{7}=sqrt(success_time_B*(1-(success_time_B/trial_B))^2/trial_B);
            success_std{9}=sqrt(success_time_C*(1-(success_time_C/trial_C))^2/trial_C);
%             success_std{12}=sqrt(success_time_D*(1-(success_time_D/trial_D))^2/trial_D);

            %%% plot confusion matrix
%             result=[success_time_A,predictA_to_B,predictA_to_C,predictA_to_D;
%                     predictB_to_A,success_time_B,predictB_to_C,predictB_to_D;
%                     predictC_to_A,predictC_to_B,success_time_C,predictC_to_D;
%                     predictD_to_A,predictD_to_B,predictD_to_C,success_time_D;];
            result=[success_time_A,predictA_to_B,predictA_to_C;
                    predictB_to_A,success_time_B,predictB_to_C;
                    predictC_to_A,predictC_to_B,success_time_C];
%             result=[trial_A,0,0;
%                     1,2,2;
%                     0,0,trial_C];
            %%%normalize the reult
            result(1,:)=100*result(1,:)/trial_A;
            result(2,:)=100*result(2,:)/trial_B;
            result(3,:)=100*result(3,:)/trial_C;
%             result(4,:)=100*result(4,:)/trial_D;
            %%% plot it
            figure
%             imagesc([2 4 6 8],[2 4 6 8],result);colorbar;
%             xticks([2 4 6 8]);xticklabels({'2','7','9','12'});
%             yticks([2 4 6 8]);yticklabels({'2','7','9','12'});
            imagesc([2 4 6],[2 4 6],result);colorbar;
            xticks([2 4 6]);xticklabels({'apple','banana','orange'});
            yticks([2 4 6]);yticklabels({'apple','banana','orange'});
            title('classify confusion matrix for all fruit')
            
            %% get finial success rate
            A=[spectralRadius_0_1_kind numInternalUnits_kind TW_kind];
            fprintf('the spectralRadius_0_1_kind %d, numInternalUnits_kind %d, TW_kind %d,',A)

            for TW_num=[2 7 9]
            A=[TW_num success_rate{TW_num}(1)*100 success_std{TW_num}*100];
            fprintf('%d: %.1f%% std %.1f%%; ',A)%the success rate of
            end

            % !echo the success rate for_each_trial
            % for i=1:length(TW_success_for_each_trial)
            %     fprintf('%.1f%%,%.1f%%;  ',TW_success_for_each_trial(i),success_rate_test_for_each_trial(i))
            % end
            % fprintf('\n')
            % 
            % !echo the calculate time for_each_trial
            % for i=1:length(test_time)
            %     fprintf('%.1fms, ',test_time{i}*1000)
            % end
            % fprintf('\n')
            fprintf('Average success rate %.1f%% and calculate time %.1fms for each trial. \n',mean(TW_success_for_each_trial),mean(cell2mat(test_time)*1000))

        end
    end
end

%     fclose(fid);


%% write ESN in to a txt file
% !echo Are you want write ESN?
% pause
% 
% file=[path_of_load_model 'ESN_oldF2Vmodel_robot.txt'];
% nfp=nForgetPoints;   %%% nForgetPoints
% a=writeESNtofile(trainedEsn, file, nfp)
% if a
%     !echo Write ESN success!
% else
%     !echo Write ESN Error!
% end

