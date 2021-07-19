clear all;close all;clc;

% the echo-state-networks repository is a submodule in this CITRIFIED
% repository, make sure you have initialized and updated it.
addpath(genpath(fullfile('.','echo-state-networks','lib','ESNToolbox')));

%% ESN classes
classes = {'apple','banana','orange','prune'};
colors = {'g','y','r','b'};

%% data set
% loads cell array 'all_timewindows' from desired data set
data_path = fullfile('..','..','data','raw_data','april','esn_finish_test');
data_set = 'timewindows.mat';
load(fullfile(data_path, data_set));
insertion_data = unpack_timewindows(all_timewindows)';

clear all_timewindows

%% define paths
% if isunix
%     path_of_load = ['../../data/segmented_data/' data_of_exp '/'];
%     path_of_save = ['../../data/esn_data/' data_of_exp '/'];
% else
%     path_of_load = ['.\data\processed_data\' data_of_exp_input '\'];
%     path_of_save = ['.\data\data_set4learn\' data_of_exp_output '\'];
% end
% if ~isfolder(path_of_save)
%     mkdir(path_of_save); 
% end

%% training parameters
test_train_rate = 0.1;
nb_restarts_training = 10;

%% grid search
% define the grid for the 4 parameters spectral_radius, nb_internal_units,
% nb_forget_points, tw. For each tw, you need to specify one overlap.
spectral_radius_grid = [0.1];
nb_internal_units_grid = [400];
nb_forget_points = [0];

for spectral_radius=spectral_radius_grid
    for nb_internal_units=nb_internal_units_grid
        for nb_forget_point=nb_forget_points 
            %% ESN
            % generate an esn 
            disp('Generating ESN ............');
            esn = generate_esn(7, nb_internal_units, length(classes), ...
                'spectralRadius', spectral_radius, 'learningMode', 'offline_multipleTimeSeries', ...
                'reservoirActivationFunction', 'tanh', 'type', 'plain_esn', ...
                'outputActivationFunction', 'identity','inverseOutputActivationFunction','identity'); 

            train_success = zeros(1, nb_restarts_training);
            test_success = zeros(1, nb_restarts_training);
            best_test_success = 0;
            for restart=1:nb_restarts_training
                cv = cvpartition(length(insertion_data), 'HoldOut', test_train_rate);
                train_idx = cv.training(1);
                test_idx = cv.test(1);

                train_set = insertion_data(train_idx);
                test_set = insertion_data(test_idx);
                
                train_input = [];
                train_output = [];
                train_output_labels = [];
                for j = 1:length(train_set)
                    input = train_set{j}.input;
                    train_input{end+1} = [input.depth, input.velocity_x, input.velocity_z ...
                                          input.force_x, input.force_z, ...
                                          input.force_derivative_x, input.force_derivative_z];
                    output = zeros(1, esn.nOutputUnits);
                    output(train_set{j}.real_class_index + 1) = 1;
                    train_output{end+1} = output;
                    train_output_labels(end+1) = train_set{j}.real_class_index + 1;
                end

                test_input = [];
                test_output = [];
                test_output_labels = [];
                for j = 1:length(test_set)
                    input = test_set{j}.input;
                    test_input{end+1} = [input.depth, input.velocity_x, input.velocity_z ...
                                          input.force_x, input.force_z, ...
                                          input.force_derivative_x, input.force_derivative_z];
                    output = zeros(1, esn.nOutputUnits);
                    output(test_set{j}.real_class_index + 1) = 1;
                    test_output{end+1} = output;
                    test_output_labels(end+1) = train_set{j}.real_class_index + 1;
                end
                clear j output
                
                [trained_esn, predicted_train_output, predicted_test_output, ~, ~] = ...
                        esn_train_and_predict(esn, train_input, train_output, test_input, nb_forget_point);

                [~, predicted_train_indices, train_scores, ~] = classify_and_evaluate(train_output_labels, predicted_train_output, 3);
                [~, predicted_test_indices, test_scores, ~] = classify_and_evaluate(test_output_labels, predicted_test_output, 3);
                train_success(restart) = train_scores.success_rate;
                test_success(restart) = test_scores.success_rate;

                if test_success(restart) > best_test_success
                    best_test_success = test_success(restart);
                    best_trained_esn = trained_esn;
                end
            end
            figure;
            plot(train_success,'b-*')
            hold on; grid on;
            plot(test_success,'r-o')
            legend('train success','test success')
            
            C = confusionmat([train_output_labels test_output_labels], [predicted_train_indices predicted_test_indices]);
            C_norm = 100 * C ./ sum(C,1);
            figure;
            imagesc([2 4 6 8],[2 4 6 8],C_norm);colorbar;
            figure;
            confusionchart(C)
        end
    end
end

%%
function unpacked=unpack_timewindows(timewindows)
timewindows = timewindows(~cellfun(@isempty, timewindows));
unpacked = [];
for i = 1:length(timewindows)
    for j = 1:3
        unpacked{end+1} = timewindows{i}{j};
    end
end
end