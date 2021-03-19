clear all;close all;clc;
% the echo-state-networks repository is a submodule in this CITRIFIED
% repository, make sure you have initialized and updated it.
if isunix
    addpath(genpath('./echo-state-networks/lib/ESNToolbox'))
else
    addpath(genpath('.\echo-state-networks\lib\ESNToolbox'))
end

%% choose data set
% define which folder should be loaded and what data is in it
data_of_exp = 'february_new';
desired_phases = 1:2;
fruits = {'apple', 'banana', 'orange'};

%% define paths
if isunix
    path_of_load = ['../../data/segmented_data/' data_of_exp '/'];
    path_of_save = ['../../data/esn_data/' data_of_exp '/'];
else
    path_of_load = ['.\data\processed_data\' data_of_exp_input '\'];
    path_of_save = ['.\data\data_set4learn\' data_of_exp_output '\'];
end
if ~isfolder(path_of_save)
    mkdir(path_of_save); 
end

%% training parameters
test_train_rate = 0.1;
nb_restarts_training = 10;

%% grid search
% define the grid for the 4 parameters spectral_radius, nb_internal_units,
% nb_forget_points, tw. For each tw, you need to specify one overlap.
spectral_radius_grid = [0.1];
nb_internal_units_grid = [100];
nb_forget_points = [0];
tw_grid = 1e-3*[50];
overlap = 1e-3 * 10;

for spectral_radius=spectral_radius_grid
    for nb_internal_units=nb_internal_units_grid
        for nb_forget_point=nb_forget_points
            for tw_idx=1:length(tw_grid)
                tw = tw_grid(tw_idx);
                overlap = overlap(tw_idx);
                
                insertion_data = [];
                insertion_labels = [];
                for fruit=1:length(fruits)
                    fruit_name = fruits{fruit};
                    %% load folder content
                    if isunix
                        dir_content = struct2cell(dir([path_of_load fruit_name '/*.csv']'));
                    else
                        dir_content = struct2cell(dir([path_of_load fruit_name '/*.csv']'));
                    end
                    filenames = dir_content(1,:);
                    clear dir_content

                    for trial=1:length(filenames)        
                        if isunix
                            filename=[path_of_load fruit_name '/' filenames{trial}];
                        else
                            filename=[path_of_load fruit_name '\' filenames{trial}];
                        end

                        data = readtable(filename);
                        sampling_frequency = (size(data, 1) - 1) / (data.timestamp(end) - data.timestamp(1));

                        %% fliter and get derivative 
                        freq = 9.380037948039710e+02; % this is the sampling frequency of the raw data from february
                        [b, a] = butter(4, 6/freq*2); % create a Butterworth 1D filter with cutoff frequency 6Hz
                                                      % 6/freq*2 is the normalized cutoff frequency

                        ee_force_filt = [filter(b, a, data.ee_force_x) filter(b, a, data.ee_force_y) filter(b, a, data.ee_force_z)];
                        ee_twist_lin_filt = [filter(b, a, data.ee_twist_lin_x) filter(b, a, data.ee_twist_lin_y) filter(b, a, data.ee_twist_lin_z)];
                        force_dot = (ee_force_filt(3:end,:) - ee_force_filt(1:end-2,:)) * sampling_frequency / 2;
                        force_dot = [zeros(2,3); force_dot];

                        %% get desired phases
                        timestamp = [];
                        signal = [];
                        for phase=desired_phases
                            idx = find(data.phase == phase);
                            timestamp = [timestamp; data.timestamp(idx)];
                            signal = [signal; ee_twist_lin_filt(idx,1:2:3) ee_force_filt(idx,1:2:3) force_dot(idx,1:2:3)]; 
                        end

                        clear data sampling_freq ee_force_filt ee_twist_lin_filt force_dot idx

                        %% sliding window
                        datapoints_per_tw = tw * length(timestamp) / (timestamp(end) - timestamp(1));
                        datapoints_overlap = overlap * length(timestamp) / (timestamp(end) - timestamp(1));
                        delay_between_tw = round(datapoints_per_tw - datapoints_overlap);
                        datapoints_per_tw = round(datapoints_per_tw);

                        labels = zeros(datapoints_per_tw, length(fruits));
                        labels(:,fruit) = 1;
                        input = [];
                        output = [];
                        for i=1:delay_between_tw:length(timestamp)-datapoints_per_tw
                            input{floor(i/delay_between_tw)+1} = signal(i:i+datapoints_per_tw,:);
                            output{floor(i/delay_between_tw)+1} = labels;
                        end                      
                        insertion_data{fruit,trial} = input;
                        insertion_labels{fruit,trial} = output;

                        clear datapoints_per_tw datapoints_overlap delay_between_tw datapoints_per_tw input output i
                    end
                end            
                insertion_data(cellfun(@isempty,insertion_data))=[];
                insertion_labels(cellfun(@isempty,insertion_labels))=[];

                %% ESN
                % generate an esn 
                disp('Generating ESN ............');
                esn = generate_esn(6, nb_internal_units, length(fruits), ...
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

                    train_set_input = insertion_data(train_idx);
                    train_set_output = insertion_labels(train_idx);
                    test_set_input = insertion_data(test_idx);
                    test_set_output = insertion_labels(test_idx);

                    [trained_esn, predicted_train_output, predicted_test_output, test_time_mean, test_time_std] = ...
                            esn_train_and_predict(esn,  horzcat(train_set_input{:}), ...
                                                  horzcat(train_set_output{:}), horzcat(test_set_input{:}), ...
                                                  nb_forget_point);

                    [~, train_scores, ~] = classify_and_evaluate(horzcat(train_set_output{:}), predicted_train_output, 3);
                    [~, test_scores, ~] = classify_and_evaluate(horzcat(test_set_output{:}), predicted_test_output, 3);
                    train_success(restart) = train_scores.success_rate;
                    test_success(restart) = test_scores.success_rate;

                    if test_success(restart) > best_test_success
                        best_test_success = test_success(restart);
                        best_trained_esn = trained_esn;
                    end
                end
                figure
                plot(train_success,'b-*')
                hold on; grid on;
                plot(test_success,'r-o')
                legend('train success','test success')
                
            end
        end
    end
end