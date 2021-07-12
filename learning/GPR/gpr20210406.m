clear all; close all; clc

load('straight_cuts_apple');
straight_cuts{3} = {};
straight_cuts = straight_cuts(~cellfun('isempty',straight_cuts));


freq = 9.380037948039710e+02; % this is the sampling frequency of the raw data from february
[b, a] = butter(4, 6/freq*2); % create a Butterworth 1D filter with cutoff frequency 6Hz
                              % 6/freq*2 is the normalized cutoff frequency

%% discard first 30ms
for cut=1:length(straight_cuts)
    data = straight_cuts{cut};
%     tmp = data(:,2);
    data(:,2) = filtfilt(b,a,data(:,2));
%     figure
%     plot(data(:,1),tmp)
%     hold on;
%     plot(data(:,1),data(:,2))
    data(:,1) = data(:,1) - data(1,1);
%     data(end,1)
    data(data(:,1) < 0.04,:) = [];
    depth_derivative = diff(data(:,2))/mean(diff(data(:,1)));
    depth_derivative = [depth_derivative(1); depth_derivative];
    straight_cuts{cut} = [data depth_derivative];
end

all_data = [];
for cut=1:2:length(straight_cuts)
    all_data = [all_data; straight_cuts{cut}];
end
all_data(:,[1 3 4 5 7 10]) = [];

%% GPR
X_train = all_data(:,2:end);
y_train = all_data(:,1);

% model_options = {'BasisFunction' 'none' 'PredictMethod' 'exact' 'Standardize' true 'KernelFunction' 'rationalquadratic' 'OptimizeHyperparameters' {'Sigma'}};
model_options = {'BasisFunction' 'none' 'PredictMethod' 'exact' 'Standardize' true 'KernelFunction' 'rationalquadratic' 'Sigma' 0.05};
gpr_model = fitrgp(X_train, y_train, model_options{:});
% load('gpr.mat')
resubLoss(gpr_model)

% [y_pred, std_pred] = resubPredict(gpr_model);
% figure(1)
% plot(X_train(:,2), y_pred, 'k.', 'LineWidth', 2.5); hold on;
% plot(X_train(:,2), y_train, 'b.', 'LineWidth', 2.5); hold on;
% hold on;grid on;
% plot(X_train(:,2), y_pred+std_pred, 'r.', 'LineWidth', 2); hold on;
% plot(X_train(:,2), y_pred-std_pred, 'r.', 'LineWidth', 2, 'HandleVisibility','off'); hold on;
% legend_str{1} = 'GPR model \mu';
% legend_str{2} = 'GPR model \sigma';

%%
figure;
hold on;grid on;
for cut=10:10
    data = straight_cuts{cut};
    data(:,1) = data(:,1) - data(1,1);
    data(data(:,1) < 0.04,:) = [];
    data(:,[1 3 4 5 7 10]) = [];
    plot3(data(:,2), data(:,4), data(:,1));
    [y_pred, std_pred] = gpr_model.predict(data(:,2:end));
    plot3(data(:,2), data(:,4), y_pred, 'k');
    plot3(data(:,2), data(:,4), y_pred+std_pred, 'r-', 'LineWidth', 2);
    plot3(data(:,2), data(:,4), y_pred-std_pred, 'r-', 'LineWidth', 2);
end
xlabel('velocity X')
ylabel('force X')
zlabel('depth')

figure;
hold on;grid on;
for cut=10:10
    data = straight_cuts{cut};
    data(:,1) = data(:,1) - data(1,1);
    data(data(:,1) < 0.04,:) = [];
    data(:,[1 3 4 5 7 10]) = [];
    plot3(data(:,2), data(:,end), data(:,1));
    [y_pred, std_pred] = gpr_model.predict(data(:,2:end));
    plot3(data(:,2), data(:,end), y_pred, 'k');
    plot3(data(:,2), data(:,end), y_pred+std_pred, 'r-', 'LineWidth', 2);
    plot3(data(:,2), data(:,end), y_pred-std_pred, 'r-', 'LineWidth', 2);
end
xlabel('velocity X')
ylabel('depth derivative')
zlabel('depth')


