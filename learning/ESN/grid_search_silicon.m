clear all;close all;clc;

% the echo-state-networks repository is a submodule in this CITRIFIED
% repository, make sure you have initialized and updated it.
addpath(genpath(fullfile('.','echo-state-networks')));

%% ESN classes
classes = {'no-tube','tube'};
colors = {'b','r'};

%% data set
% loads cell array 'all_timewindows' from desired data set
data_path = fullfile('..','..','data','raw_data','mai','test');
data_set = 'silicon_0512_labeled.mat';
load(fullfile(data_path, data_set));

%% settings

plot_figures = false;

save_esn = false;
esn_path = fullfile(pwd,'ESN_400_20210503.yaml');

%% training parameters
test_train_rate = 0.4;
nb_restarts_training = 20;

%% grid search
% define the grid for the 4 parameters spectral_radius, nb_internal_units,
% nb_forget_points, tw. For each tw, you need to specify one overlap.
tw_size_grid = [10 20 30 40 50];
tw_overlap_grid = [1 2 3 4 5];
spectral_radius_grid = [0.1 0.5 0.9];
nb_internal_units_grid = [100 150 200 300 400];
nb_forget_points = [0 1 2 3];
performances = [];

for tw=1:length(tw_size_grid)
    tw_size = tw_size_grid(tw);
    tw_overlap = tw_overlap_grid(tw);
    
    insertion_data = {};
    for trial=1:length(labeled)
        data = labeled{trial};
        % downsample to 500Hz = drop every second datapoint
        data(1:2:end,:) = [];
        % put into time windows
        time_tw = buffer(data(:,1), tw_size, tw_overlap, 'nodelay');
        velocity_z_tw = buffer(data(:,3), tw_size, tw_overlap, 'nodelay');
        force_z_tw = buffer(data(:,4), tw_size, tw_overlap, 'nodelay');
        label_tw = buffer(data(:,5), tw_size, tw_overlap, 'nodelay');
        % drop last column because incomplete
        time_tw(:,end) = [];
        velocity_z_tw(:,end) = [];
        force_z_tw(:,end) = [];
        label_tw(:,end) = [];
        for i=1:size(time_tw, 2)
            % get mean delta t
            dt = mean(diff(time_tw(:,i)));
            acc_z = (velocity_z_tw(3:end,i) - velocity_z_tw(1:end-2,i)) / 2*dt;
            acc_z = [acc_z(1); acc_z; acc_z(end)];
            fd_z = (force_z_tw(3:end,i) - force_z_tw(1:end-2,i)) / 2*dt;
            fd_z = [fd_z(1); fd_z; fd_z(end)];
            s.input.acceleration_z = acc_z;
            s.input.force_derivative_z = fd_z;
            s.real_class_index = round(mean(label_tw(:,i)));
            insertion_data{end+1} = s;
            if s.real_class_index == 2
                continue
            end
        end
    end
    insertion_data = insertion_data';
    
    for spectral_radius=spectral_radius_grid
        for nb_internal_units=nb_internal_units_grid
            for nb_forget_point=nb_forget_points 
                %% ESN
                % generate an esn 
                disp('Generating ESN ............');
                esn = generate_esn(2, nb_internal_units, length(classes), ...
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
                        train_input{end+1} = [input.acceleration_z, input.force_derivative_z];
                        output = zeros(1, esn.nOutputUnits);
                        output(train_set{j}.real_class_index + 1) = 1;
                        train_output{end+1} = repmat(output, length(input.acceleration_z), 1);
                        train_output_labels(end+1) = train_set{j}.real_class_index + 1;
                    end

                    test_input = [];
                    test_output = [];
                    test_output_labels = [];
                    for j = 1:length(test_set)
                        input = test_set{j}.input;
                        test_input{end+1} = [input.acceleration_z, input.force_derivative_z];
                        output = zeros(1, esn.nOutputUnits);
                        output(test_set{j}.real_class_index + 1) = 1;
                        test_output{end+1} = repmat(output, length(input.acceleration_z), 1);
                        test_output_labels(end+1) = test_set{j}.real_class_index + 1;
                    end
                    clear j output

                    [trained_esn, predicted_train_output, predicted_test_output, ~, ~] = ...
                            esn_train_and_predict(esn, train_input, train_output, test_input, nb_forget_point);

                    [~, predicted_train_indices, train_scores, ~] = classify_and_evaluate(train_output_labels, predicted_train_output, 3);
                    [~, predicted_test_indices, test_scores, ~] = classify_and_evaluate(test_output_labels, predicted_test_output, 3);
                    train_success(restart) = train_scores.success_rate;
                    test_success(restart) = test_scores.success_rate;

                    if test_success(restart) > best_test_success
                        best_result.test_success = test_success(restart);
                        best_result.train_labels = train_output_labels;
                        best_result.test_labels = test_output_labels;
                        best_result.predicted_train_labels = predicted_train_indices;
                        best_result.predicted_test_labels = predicted_test_indices;
                        best_result.trained_esn = trained_esn;
                        best_result.train_scores = train_scores;
                        best_result.test_scores = test_scores;
                        best_result.nb_forget_points = nb_forget_point;
                    end
                end

                if plot_figures
                    figure(1);
                    plot(train_success,'b-*')
                    hold on; grid on;
                    plot(test_success,'r-o')
                    legend('train success','test success')

                    C = confusionmat([best_result.train_labels best_result.test_labels], [best_result.predicted_train_labels best_result.predicted_test_labels]);
                    C_norm = 100 * C ./ sum(C,2);

                    figure(2);
                    cm = confusionchart(C, classes);
                end

                performance.spectral_radius = spectral_radius;
                performance.nb_forget_points = nb_forget_point;
                performance.best_result = best_result;
                performance.mean_test_success = mean(test_success);
                performances{end+1} = performance;
            end
        end
    end
end
    
if save_esn
    writeESNtoYAML(performance.best_result.trained_esn, esn_path, performance.best_result.nb_forget_points, classes);
end

% best_grid_result = 0;
% grid_results = [];
% for p = performances
%     grid_results = [grid_results; p{1}.spectral_radius p{1}.nb_forget_points p{1}.mean_test_success];
%     if p{1}.mean_test_success > best_grid_result
%         best_grid_result = p{1}.mean_test_success;
%         best_p = p{1};
%     end
% end
% figure;
% [x, y] = meshgrid(spectral_radius_grid, nb_forget_points);
% surf(x,y, reshape(grid_results(:,3),length(nb_forget_points), length(spectral_radius_grid)))
