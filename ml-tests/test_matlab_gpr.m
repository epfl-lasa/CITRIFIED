clear all; close all; clc;
load 'train_data.mat'
load 'test_data.mat'

X = train_data(:,1);
y = train_data(:,2);

X_test = test_data(:,1);
y_test = test_data(:,2);

%%
model_options = {'BasisFunction' 'none' 'FitMethod' 'exact' 'OptimizeHyperparameters' {'KernelFunction' 'Sigma'}};

% gpr = fitrgp(X, y, model_options{:});
% best observed sigma: 0.0001021, rotquad
% best estimated sigma: 0.00017929, rotquad
load gpr_opt_sigma_kern;
[y_pred, std_pred]= resubPredict(gpr);
figure;
plot(X, y, '.')
hold on;
plot(X, y_pred)
plot(X, y_pred + std_pred, '--')
plot(X, y_pred - std_pred, '--')
resubLoss(gpr)
loss(gpr, X_test, y_test)
% HORRIBLY OVERFITTED

%%
model_options = {'BasisFunction' 'none' 'FitMethod' 'exact' 'KernelFunction' 'squaredexponential' 'OptimizeHyperparameters' {'Sigma'}};
% gpr = fitrgp(X, y, model_options{:});
% best observed sigma: 0.43111
% best estimated sigma: 0.91296
load gpr_sqdexp_opt_sigma;
[y_pred, std_pred]= resubPredict(gpr);
figure;
plot(X, y, '.')
hold on;
plot(X, y_pred)
plot(X, y_pred + std_pred, '--')
plot(X, y_pred - std_pred, '--')
resubLoss(gpr)
loss(gpr, X_test, y_test)
% SEEMS LIKE A GOOD MODEL, std_pred is ~ 0.92, sigmaL is 0.1195, sigmaF
% 0.5782

%%
model_options = {'BasisFunction' 'none' 'FitMethod' 'exact' 'KernelFunction' 'squaredexponential' 'Sigma' 1/sqrt(2)};
gpr = fitrgp(X, y, model_options{:});
[y_pred, std_pred]= resubPredict(gpr);
figure;
plot(X, y, '.')
hold on;
plot(X, y_pred)
plot(X, y_pred + std_pred, '--')
plot(X, y_pred - std_pred, '--')
resubLoss(gpr)
loss(gpr, X_test, y_test)
% SEEMS LIKE A GOOD MODEL, same as above, std_pred is ~ 0.96, sigmaL is
% 0.1175, sigmaF 0.5589, Sigma is changed to 0.9459

%%
model_options = {'BasisFunction' 'none' 'FitMethod' 'exact' 'KernelFunction' 'squaredexponential' 'Sigma' 1/sqrt(2) 'ConstantSigma' true};
gpr = fitrgp(X, y, model_options{:});
[y_pred, std_pred]= resubPredict(gpr);
figure;
plot(X, y, '.')
hold on;
plot(X, y_pred)
plot(X, y_pred + std_pred, '--')
plot(X, y_pred - std_pred, '--')
resubLoss(gpr)
loss(gpr, X_test, y_test)
% SEEMS LIKE A GOOD MODEL, same as above, std_pred is ~ 0.96, sigmaL is
% 0.1352, sigmaF 0.7361

%% sigmaL is the "length scale, default = std(X)
%% sigmaF is the signal standard deviation, default = std(y)/sqrt(2)
%% Sigma is "inital value for the noise standard deviation"



