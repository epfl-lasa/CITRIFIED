%% GPR train single model
% Dominic Reber
% 04.01.2021

clc; clearvars; close all;
addpath(fullfile('functions'));

% Data choice
experiment = 'december';
fruit = 'orange';
cut_quality = 'good';

% Learning parameter choice
TRAIN_PERCENTAGE = 0.7;
NB_TRAIN_SAMPLES = 2000;

OPTIMIZE = false;       % Hyperparameter optimization

%% Segmented data reading
data_folder = fullfile('..', 'data', 'segmented_data', experiment, fruit, cut_quality);
data_files = dir(data_folder);
data_files = data_files(~[data_files.isdir]);

nb_train = round(TRAIN_PERCENTAGE * length(data_files));
nb_test = length(data_files) - nb_train;
idx = randperm(length(data_files));

train_set = cell(nb_train,1);
for i=1:nb_train
    X_train = import_csv_as_table(fullfile(data_folder, data_files(idx(i)).name), {'displacement'});
    y_train = import_csv_as_table(fullfile(data_folder, data_files(idx(i)).name), {'force_x', 'force_y', 'force_z'});
    [X_train, y_train] = downsample_data(X_train, y_train, round(NB_TRAIN_SAMPLES/nb_train));
    train_set{i} = [X_train, y_train];
end

test_set = cell(nb_test,1);
for i=1:nb_test
    X_test = import_csv_as_table(fullfile(data_folder, data_files(idx(nb_train+i)).name), {'displacement'});
    y_test = import_csv_as_table(fullfile(data_folder, data_files(idx(nb_train+i)).name), {'force_x', 'force_y', 'force_z'});
    [X_test, y_test] = downsample_data(X_test, y_test, round(round(NB_TRAIN_SAMPLES/TRAIN_PERCENTAGE)-NB_TRAIN_SAMPLES/nb_test));
    test_set{i} = [X_test, y_test];
end
clear X_train y_train X_test y_test; 


X_train_vector = []; y_train_vector = [];
for i=1:nb_train
    X_train_vector = [X_train_vector; train_set{i}.displacement];
    y_train_vector = [y_train_vector; train_set{i}.force_x];
end
[X_train_vector, I] = sort(X_train_vector);
y_train_vector = y_train_vector(I);

X_test_vector = []; y_test_vector = [];
for i=1:nb_test
    X_test_vector = [X_test_vector; test_set{i}.displacement];
    y_test_vector = [y_test_vector; test_set{i}.force_x];
end
[X_test_vector, I] = sort(X_test_vector);
y_test_vector = y_test_vector(I);

%% Choose the correct options for plotting and training model
% rng('default')
if (OPTIMIZE == false)
    model_options = {'BasisFunction' 'none' 'PredictMethod' 'exact' 'Standardize' false 'KernelFunction' 'matern32' 'Sigma' std(y_train_vector)};
%     model_options = {'BasisFunction' 'none' 'PredictMethod' 'exact' 'Standardize' false 'KernelFunction' 'rationalquadratic' 'Sigma' std(y_train_vector)};
else
    model_options = {'BasisFunction' 'none' 'PredictMethod' 'exact' 'Standardize' false 'KernelFunction' 'matern32' 'OptimizeHyperparameters' {'Sigma'}};
%     model_options = {'BasisFunction' 'none' 'PredictMethod' 'exact' 'Standardize' false 'OptimizeHyperparameters' {'KernelFunction' 'Sigma'}};
end
% model_options = {'PredictMethod' 'exact' 'OptimizeHyperparameters' 'all'};



%% Train the GPR model using imported and sorted data
gpr_model = fitrgp(X_train_vector, y_train_vector, model_options{:});
test_loss = loss(gpr_model, X_test_vector, y_test_vector)

%% Plot the model along the testing set
% Get the mean and variance of the model
[y_pred, std_pred]= resubPredict(gpr_model);

% Plot them on a figure
figure(1)
clf(figure(1))
plot(1e3*X_train_vector, y_pred, 'k', 'LineWidth', 2.5); hold on;
plot(1e3*X_train_vector, y_pred+std_pred, 'r-', 'LineWidth', 2); hold on;
plot(1e3*X_train_vector, y_pred-std_pred, 'r-', 'LineWidth', 2, 'HandleVisibility','off'); hold on;
legend_str{1} = 'GPR model \mu';
legend_str{2} = 'GPR model \sigma';

% Plot the train trajectories along the model
% for i = 1:nb_train
%     figure(1)
%     plot(1e3*train_set{i}.displacement, train_set{i}.force_x, 'LineWidth', .5, 'HandleVisibility','off'); hold on;
% end

% Plot the test trajectories along the model
for i = 1:nb_test
    figure(1)
    plot(1e3*test_set{i}.displacement, test_set{i}.force_x, 'LineWidth', 1); hold on;
    legend_str{i+2} = strcat('Test traj: ', num2str(i));
end

% Figure titles and axes
figure(1);
xlabel('Displacement (mm)','interpreter','latex');
ylabel('Cutting force (N)','interpreter','latex');
grid minor;
set(gca,'TickLabelInterpreter','latex');
set(gca,'FontSize',18);
set(gca,'LineWidth',1.5);
graph_title = ['GPR Model ; Train ' num2str(TRAIN_PERCENTAGE) ' Test ' num2str(1-TRAIN_PERCENTAGE) ' ; Config ' fruit, '-' cut_quality];
title(graph_title, 'interpreter','latex');
legend(legend_str, 'Location', 'Best')
