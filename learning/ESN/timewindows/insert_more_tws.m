clear all;close all;clc;
addpath(genpath('./echo-state-networks/lib/ESNToolbox'))

%% choice data set
data_of_exp = 'march_esn_test';
path_of_load = ['../../data/raw_data/' data_of_exp '/'];
path_of_save = ['../../data/raw_data/' data_of_exp '/two_more_tws/'];
if ~isfolder(path_of_save)
    mkdir(path_of_save); 
end

%% load esn
load('ESN_march30_900.mat');
classNames = {'apple','banana','orange','prune'};

%% loop through files
dir_content = struct2cell(dir([path_of_load '*.json']'));

nb_splits = 3;
for nb_file=1:size(dir_content, 2)
    nb_file
    start_time_1 = 0;
    start_time_2 = 0;
    additional_tw_1 = [];
    additional_tw_2 = [];
    skip_1 = 2;
    skip_2 = 2;
    
    in = fopen([path_of_load dir_content{1,nb_file}], 'r');
    out = fopen([path_of_save dir_content{1,nb_file}], 'w');
    message = jsondecode(fgetl(in));
    fwrite(out,[jsonencode(message) sprintf('\r')]);
    while ~feof(in)
        message = jsondecode(fgetl(in));
        if ~strcmp(message.control.phase, 'insertion')
            fwrite(out,[jsonencode(message) sprintf('\r')]);
            continue
        end
        skip_1 = skip_1 - 1;
        skip_2 = skip_2 - 1;
        if ~start_time_1 && ~start_time_2
            start_time_1 = message.time;
            start_time_2 = message.time;
        else
            if message.time - start_time_1 > 0.083 && skip_1 <= 0
                skip_1 = 2;
                additional_tw_1 = [additional_tw_1; getSignals(message)];
            end
            if message.time - start_time_2 > 0.16 && skip_2 <= 0
                skip_2 = 2;
                additional_tw_2 = [additional_tw_2; getSignals(message)];
            end
        end   
        if size(additional_tw_1,1) >= 50
            dt = mean(diff(additional_tw_1(:,1)));
            additional_tw_1(2:end-1,7:end) = (additional_tw_1(3:end,5:6) - additional_tw_1(1:end-2,5:6)) / (2*dt);
            additional_tw_1(1,7:end) = additional_tw_1(2,7:end);
            additional_tw_1(end,7:end) = additional_tw_1(end-1,7:end);
            
            predicted_output = test_esn(additional_tw_1(:,2:end), trainedEsn, 0);
            
            time_window = floor(size(predicted_output) / nb_splits);
            avg_predicted_output = zeros(nb_splits , trainedEsn.nOutputUnits);
            for j=1:nb_splits-1
                avg_predicted_output(j,:) = mean(predicted_output((j-1)*time_window+1:j*time_window,:));
            end
            avg_predicted_output(nb_splits,:) = mean(predicted_output((nb_splits-1)*time_window+1:end, :));
            sum_avg_predicted_output = sum(avg_predicted_output);
            normalized_predicted_output = sum(avg_predicted_output) / sum(sum(avg_predicted_output));

            [~, ind] = max(normalized_predicted_output);
            predicted_class = zeros(1,trainedEsn.nOutputUnits);
            predicted_class(ind) = 1;
            class = classNames{ind};
            
            message.esn.input.time = additional_tw_1(:,1);
            message.esn.input.depth = additional_tw_1(:,2);
            message.esn.input.velocity_x = additional_tw_1(:,3);
            message.esn.input.velocity_z = additional_tw_1(:,4);
            message.esn.input.force_x = additional_tw_1(:,5);
            message.esn.input.force_z = additional_tw_1(:,6);
            message.esn.input.force_derivative_x = additional_tw_1(:,7);
            message.esn.input.force_derivative_z = additional_tw_1(:,8);
            message.esn.class_index = ind-1;
            message.esn.probabilities = normalized_predicted_output;
            message.esn.class_name = class;
            
            start_time_1 = start_time_1 + 0.25;
            additional_tw_1 = [];
        end
        if size(additional_tw_2,1) >= 50
            dt = mean(diff(additional_tw_2(:,1)));
            additional_tw_2(2:end-1,7:end) = (additional_tw_2(3:end,5:6) - additional_tw_2(1:end-2,5:6)) / (2*dt);
            additional_tw_2(1,7:end) = additional_tw_2(2,7:end);
            additional_tw_2(end,7:end) = additional_tw_2(end-1,7:end);
            
            predicted_output = test_esn(additional_tw_2(:,2:end), trainedEsn, 0);
            
            time_window = floor(size(predicted_output) / nb_splits);
            avg_predicted_output = zeros(nb_splits , trainedEsn.nOutputUnits);
            for j=1:nb_splits-1
                avg_predicted_output(j,:) = mean(predicted_output((j-1)*time_window+1:j*time_window,:));
            end
            avg_predicted_output(nb_splits,:) = mean(predicted_output((nb_splits-1)*time_window+1:end, :));
            sum_avg_predicted_output = sum(avg_predicted_output);
            normalized_predicted_output = sum(avg_predicted_output) / sum(sum(avg_predicted_output));

            [~, ind] = max(normalized_predicted_output);
            predicted_class = zeros(1,trainedEsn.nOutputUnits);
            predicted_class(ind) = 1;
            class = classNames{ind};
            
            message.esn.input.time = additional_tw_2(:,1);
            message.esn.input.depth = additional_tw_2(:,2);
            message.esn.input.velocity_x = additional_tw_2(:,3);
            message.esn.input.velocity_z = additional_tw_2(:,4);
            message.esn.input.force_x = additional_tw_2(:,5);
            message.esn.input.force_z = additional_tw_2(:,6);
            message.esn.input.force_derivative_x = additional_tw_2(:,7);
            message.esn.input.force_derivative_z = additional_tw_2(:,8);
            message.esn.class_index = ind-1;
            message.esn.probabilities = normalized_predicted_output;
            message.esn.class_name = class;
            
            start_time_2 = start_time_2 + 0.25;
            additional_tw_2 = [];
        end
%         if isfield(message, 'esn')           
%             input = message.esn.input;
%             input = [input.depth input.velocity_x input.velocity_z input.force_x ...
%                      input.force_z input.force_derivative_x input.force_derivative_z];
%             predicted_output = test_esn(input, trainedEsn, 0);
%             time_window = floor(size(predicted_output) / nb_splits);
%             avg_predicted_output = zeros(nb_splits , trainedEsn.nOutputUnits);
%             for j=1:nb_splits-1
%                 avg_predicted_output(j,:) = mean(predicted_output((j-1)*time_window+1:j*time_window,:));
%             end
%             avg_predicted_output(nb_splits,:) = mean(predicted_output((nb_splits-1)*time_window+1:end, :));
%             sum_avg_predicted_output = sum(avg_predicted_output);
%             normalized_predicted_output = sum(avg_predicted_output) / sum(sum(avg_predicted_output));
% 
%             [~, ind] = max(normalized_predicted_output);
%             predicted_class = zeros(1,trainedEsn.nOutputUnits);
%             predicted_class(ind) = 1;
%             class = classNames{ind};
%         end
        fwrite(out,[jsonencode(message) sprintf('\r')]);
    end
    fclose(in);
    fclose(out);
end

%%
function signals = getSignals(message)
    signals = zeros(1,8);
    signals(1,1) = message.time;
    signals(1,2) = message.model.depth;
    for i=1:length(message.filtered.bodies)
        if strcmp(message.filtered.bodies{i}.name, 'ee') && strcmp(message.filtered.bodies{i}.frame, 'ee')
            signals(1,3:4) = message.filtered.bodies{i}.twist.linear(1:2:3)';
        elseif strcmp(message.filtered.bodies{i}.name, 'ft_sensor') && strcmp(message.filtered.bodies{i}.frame, 'robot')
            signals(1,5:6) = message.filtered.bodies{i}.wrench.force(1:2:3)';
        end
    end
end
