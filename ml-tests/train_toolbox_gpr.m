%% Data
clear all; close all; clc;
load 'train_data.mat'
load 'test_data.mat'

X = train_data(:,1);
y = train_data(:,2);

X_test = test_data(:,1);
y_test = test_data(:,2);

% Plot data
options             = [];
options.points_size = 10;
options.plot_labels = {'x','y'};
options.title       = 'Standardized data'; 

% if exist('h1','var') && isvalid(h1), delete(h1);end
% h1      = ml_plot_data([X(:),y(:)],options);

%% Train GPR

meanfunc = {@meanZero};
covfunc  = {@covSEiso}; 

ell      = 0.1175;  % kernel width of RBF covariance function.
sf       = 0.9459;  % signal variance (not measurement noise)
sn       = 0.5589;  % measurement noise


hyp      = [];
hyp.cov  = log([ell; sf]);
hyp.lik  = log(sn);


% Test GP [gpml toolbox]
[m,s2] = gp(hyp, @infExact, meanfunc, covfunc, @likGauss, X, y, X_test);

if exist('h2','var') && isvalid(h2), delete(h2);end
h2 = figure;

f = [m+2*sqrt(s2); flipdim(m-2*sqrt(s2),1)];
fill([X_test; flipdim(X_test,1)], f, [7 7 7]/8);
hold on;

options             = [];
options.no_figure   = true;
options.points_size = 10;
% ml_plot_data([X(:),y(:)],options);

% THIS REPRODUCES THE EXAMPLE 3 FROM TEST_MATLAB_GPR MORE OR LESS WELL


