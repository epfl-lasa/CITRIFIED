clear all;close all;clc;
addpath(genpath('./echo-state-networks/lib/ESNToolbox'))

%% choice data set
data_of_exp = 'march_esn_test';
path_of_load = ['../../../data/raw_data/' data_of_exp '/'];
path_of_save = ['../../../data/raw_data/' data_of_exp '/one_more_tws/'];
if ~isfolder(path_of_save)
    mkdir(path_of_save); 
end

%% load esn
load('../mat_files/ESN_march30_900.mat');
classNames = {'apple','banana','orange','prune'};

%% loop through files
dir_content = struct2cell(dir([path_of_load '*.json']'));

nb_splits = 3;
for nb_file=1:size(dir_content, 2)
    nb_file
    start_time = 0;
    additional_tw = [];
    skip = 2;
    
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
        skip = skip - 1;
        if ~start_time
            start_time = message.time;
        elseif message.time - start_time > 0.125 && skip <= 0
            skip = 2;
            additional_tw = [additional_tw; getSignals(message)];
        end   
        if size(additional_tw,1) >= 50
            dt = mean(diff(additional_tw(:,1)));
            additional_tw(2:end-1,7:end) = (additional_tw(3:end,5:6) - additional_tw(1:end-2,5:6)) / (2*dt);
            additional_tw(1,7:end) = additional_tw(2,7:end);
            additional_tw(end,7:end) = additional_tw(end-1,7:end);
            
            predicted_output = test_esn(additional_tw(:,2:end), trainedEsn, 0);
            
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
            
            message.esn.input.time = additional_tw(:,1);
            message.esn.input.depth = additional_tw(:,2);
            message.esn.input.velocity_x = additional_tw(:,3);
            message.esn.input.velocity_z = additional_tw(:,4);
            message.esn.input.force_x = additional_tw(:,5);
            message.esn.input.force_z = additional_tw(:,6);
            message.esn.input.force_derivative_x = additional_tw(:,7);
            message.esn.input.force_derivative_z = additional_tw(:,8);
            message.esn.class_index = ind-1;
            message.esn.probabilities = normalized_predicted_output;
            message.esn.class_name = class;
            
            start_time = start_time + 0.25;
            additional_tw = [];
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
