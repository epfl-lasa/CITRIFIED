%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Rui Wu 2020.12.17
%   show best result in grid search and plot all grid search result and
%   show best
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
%%  0 define learning model path
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

data_of_exp='february_new_learn';

%% for differend grid search
do_grid=1;

if do_grid==1
    if isunix
        %%% for linux %%%%%%%%%%%%%%%%%%%%%%%%%
        path_of_load_model = ['./data/data_set4learn/' data_of_exp '/all_train_model/10gridsearch1/'];
        path_of_plot= ['./data/figure_for_paper/' data_of_exp '/'];
    else
        path_of_load_model = ['.\data\data_set4learn\' data_of_exp '\all_train_model\10gridsearch1\'];
        path_of_plot= ['.\data\figure_for_paper\' data_of_exp '\'];
    end

    spectralRadius_0_1_kind_grid=[1:10];
    numInternalUnits_kind_grid=[1:10];
    TW_kind_grid=[1:3];
    
    spectralRadius = [0.1:0.1:1]; 
    numInternalUnits1= [100:100:1000];
    TW_kind_1=[50:50:250]*0.001;
    overlap1=[10:10:50]*0.001;
    
    spectralRadius_0_1_kind_best=spectralRadius_0_1_kind_grid;
    numInternalUnits_kind_best=numInternalUnits_kind_grid;
    TW_kind_best=TW_kind_grid;

%%%%%%%%% this one is the best, only orange ger wrong, which same with the
%%%%%%%%% data we see on C5
    spectralRadius_0_1_kind_best=1;
    numInternalUnits_kind_best=1;
    TW_kind_best=1;
    

elseif do_grid==2 
    if isunix
        %%% for linux %%%%%%%%%%%%%%%%%%%%%%%%%
        path_of_load_model = ['./data/data_set4learn/' data_of_exp '/all_train_model/10gridsearch2/'];
        path_of_plot= ['./data/figure_for_paper/' data_of_exp '/10gridsearch/'];
    else
        path_of_load_model = ['.\data\data_set4learn\' data_of_exp '\all_train_model\10gridsearch2\'];
        path_of_plot= ['.\data\figure_for_paper\' data_of_exp '\10gridsearch\'];
    end
    spectralRadius_0_1_kind_grid=[1:10];
    numInternalUnits_kind_grid=[1:10];
    TW_kind_grid=[1:3];
    spectralRadius_0_1_kind_best=1;
    numInternalUnits_kind_best=9;
    TW_kind_best=1;
end


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  choice learning parameter
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%% change the tissues number %%%%%
exp_kind_total=[1:3];

class_label={'apple','banana','orange'};

%%%%% using which direction　%%%
not_use_xy_dir_and_add_depth=1;

%%%%% how many label
label_num=3;

learning_method_name=['robot_ESN'];



%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% switch grid search
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% load learning model

%     fid = fopen([path_of_load_model 'grid_search_result.txt'],'wt');


for spectralRadius_0_1_kind=spectralRadius_0_1_kind_best  % 5 5 3

    spectralRadius_0_1=spectralRadius(spectralRadius_0_1_kind);
    
    for numInternalUnits_kind=numInternalUnits_kind_best
        
        numInternalUnits=numInternalUnits1(numInternalUnits_kind);
        
        for TW_kind=TW_kind_best
            
            time_of_timeWindow=TW_kind_1(TW_kind);%s, timeWindow is 200ms
            time_of_overlap=overlap1(TW_kind);%s, overlap is 40ms 
            
            %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %% switch different learning algorithm here
            %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %% load learning model
                load([path_of_load_model learning_method_name '_TW_' num2str(TW_kind) '_InterUnit_' num2str(numInternalUnits) '_spectralRadius_' num2str(spectralRadius_0_1_kind) '_learning_4_tissue.mat']);

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
                clearvars -except class_label numInternalUnits1 spectralRadius overlap1 TW_kind_1 label_num numInternalUnits_kind_best TW_kind_best do_grid fid learning_method_name extFeatures classfer k_fold test_train_rate with_robot_data spectralRadius_0_1 numInternalUnits time_of_timeWindow time_of_overlap learning_method_name path_of_load_model spectralRadius_0_1_kind numInternalUnits_kind TW_kind less_feature_no_vel testtimestd test_time SVMModel learning_method fail_trials TW_success_for_each_trial success_time_A trial_A success_time_B trial_B success_time_C trial_C...
                    success_time_D  trial_D test_cell nForgetPoints nOutputUnits score_validation trials_num trainedEsn success_rate_test_for_each_trial...
                    predict*
                
                %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %% ESN
                %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                disp('Testing ESN ............');
                nForgetPoints=0;
                nOutputUnits=label_num;

                for TW_num=1:size(test_cell{trials_num},2)
                    testInputSequence{TW_num}=test_cell{trials_num}{TW_num}(:,1:end-label_num);
                    testOutputSequence{TW_num}=test_cell{trials_num}{TW_num}(:,end-label_num+1:end);
                end

                calc_time=zeros(length(testInputSequence),1);

                predictedTestOutput = [];
                for TW_num=1:length(testInputSequence)
                    predictedTestOutput{TW_num} = zeros(length(testInputSequence{TW_num})-nForgetPoints, nOutputUnits);
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
                    if label_num==5
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
                    elseif label_num==4
                        if predicted_class_time_squence{TW_num}==[1 0 0 0]
                            label(TW_num)=1;
                        elseif predicted_class_time_squence{TW_num}==[0 1 0 0]
                            label(TW_num)=2;
                        elseif predicted_class_time_squence{TW_num}==[0 0 1 0]
                            label(TW_num)=7;
                        elseif predicted_class_time_squence{TW_num}==[0 0 0 1]
                            label(TW_num)=9;
                        end

                        if real_class_time_squence{TW_num}==[1 0 0 0]
                            real_label(TW_num)=1;
                        elseif real_class_time_squence{TW_num}==[0 1 0 0]
                            real_label(TW_num)=2;
                        elseif real_class_time_squence{TW_num}==[0 0 1 0]
                            real_label(TW_num)=7;
                        elseif real_class_time_squence{TW_num}==[0 0 0 1]
                            real_label(TW_num)=9;
                        end
                    elseif label_num==3
                        if predicted_class_time_squence{TW_num}==[1 0 0]
                            label(TW_num)=2;
                        elseif predicted_class_time_squence{TW_num}==[0 1 0]
                            label(TW_num)=7;
                        elseif predicted_class_time_squence{TW_num}==[0 0 1]
                            label(TW_num)=9;
                        end

                        if real_class_time_squence{TW_num}==[1 0 0]
                            real_label(TW_num)=2;
                        elseif real_class_time_squence{TW_num}==[0 1 0]
                            real_label(TW_num)=7;
                        elseif real_class_time_squence{TW_num}==[0 0 1]
                            real_label(TW_num)=9;
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

%                 for TW_num=5:length(label)
%                     label_perdict=[label(TW_num-4) label(TW_num-3) label(TW_num-2) label(TW_num-1) label(TW_num)];
%                     [out] = majorityvote(label_perdict);
%                     out1(TW_num-4)=out;
% 
%                     real_label_perdict=[real_label(TW_num-4) real_label(TW_num-3) real_label(TW_num-2) real_label(TW_num-1) real_label(TW_num)];
%                     [real_out] = majorityvote(real_label_perdict );
%                     real_out1(TW_num-4)=real_out;
%                 end
                
                for TW_num=3:length(label)
                    label_perdict=[label(TW_num-2) label(TW_num-1) label(TW_num)];
                    [out] = majorityvote(label_perdict);
                    out1(TW_num-2)=out;

                    real_label_perdict=[real_label(TW_num-2) real_label(TW_num-1) real_label(TW_num)];
                    [real_out] = majorityvote(real_label_perdict );
                    real_out1(TW_num-2)=real_out;
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
                        testInputSequence{TW_num}=test_cell{trials_num}{TW_num}(:,1:end-label_num);
                        testOutputSequence{TW_num}=test_cell{trials_num}{TW_num}(:,end-label_num+1:end);
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
            title(['classify confusion matrix for all fruit' num2str(spectralRadius_0_1_kind) num2str(numInternalUnits_kind) num2str(TW_kind)])
            
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
!echo Are you want write ESN?
pause

file_mat=[path_of_load_model 'ESN_february' num2str(spectralRadius_0_1_kind) num2str(numInternalUnits_kind) num2str(TW_kind) '.mat'];
save(file_mat,'trainedEsn');

nfp=nForgetPoints;   %%% nForgetPoints

% file=[path_of_load_model 'ESN_february' num2str(spectralRadius_0_1_kind) num2str(numInternalUnits_kind) num2str(TW_kind) '.txt'];
% a=writeESNtofile(trainedEsn, file, nfp)

file=[path_of_load_model 'ESN_february' num2str(spectralRadius_0_1_kind) num2str(numInternalUnits_kind) num2str(TW_kind) '.yaml'];
a=writeESNtoYAML(trainedEsn, file, nfp, class_label);

if a
    !echo Write ESN success!
else
    !echo Write ESN Error!
end
