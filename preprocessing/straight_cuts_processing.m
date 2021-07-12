clear all;close all;clc;

% addpath(genpath('./lib/CITRIFIED_lib'))

%% choice data set
data_of_exp = 'march_depth';
fruits = {'apple', 'orange'};

path_of_load = ['../data/raw_data/' data_of_exp '/'];

%% raw data
fruit_name = fruits{2};
dir_content = struct2cell(dir([path_of_load fruit_name '/*.csv']'));
filenames = dir_content(1,:);
clear dir_content


freq = 9.380037948039710e+02; % this is the sampling frequency of the raw data from february

[b, a] = butter(4, 6/freq*2); % create a Butterworth 1D filter with cutoff frequency 6Hz
                              % 6/freq*2 is the normalized cutoff frequency


straight_cuts = {};
cut = 1;
for trial=1:length(filenames) 
    close all
    parameters = strsplit(filenames{trial}, '_');
    if str2double(parameters{3}) < 8
        continue
    end
    if str2double(parameters{3}) == 8 && strcmp(fruit_name, 'apple')
        continue
    end
    filename = [path_of_load fruit_name '/' filenames{trial}];
    data = readtable(filename);
    
    % find where desired twist was zero during pause before cut
    zero_twist_idx = find(data.des_twist_lin_x == 0 & data.des_twist_lin_y == 0 & data.des_twist_lin_z == 0);
    if isempty(find(diff(zero_twist_idx) > 1, 1))
        start_idx = zero_twist_idx(end);
    else
        start_idx = find(diff(zero_twist_idx) > 1, 1, 'first') + zero_twist_idx(1) - 1;
    end
    cut_idx = logical([zeros(start_idx,1); ones(length(data.time) - start_idx, 1)]);
    
    nan_depth_idx = find(isnan(data.depth) & cut_idx);
    if ~isempty(nan_depth_idx)
        if isempty(find(diff(nan_depth_idx) > 1, 1))
            end_idx_depth = nan_depth_idx(1) - 1;
        else
            end_idx_depth = nan_depth_idx(find(diff(nan_depth_idx) > 1, 1, 'first') + 1) - 1;
        end
        cut_idx = logical([zeros(start_idx,1); ones(end_idx_depth - start_idx, 1); zeros(length(data.time) - end_idx_depth,1)]);
    end
    
    nan_depth_idx = find(isnan(data.depth) & cut_idx);
    cut_idx(nan_depth_idx) = false(length(nan_depth_idx), 1);
    
    positive_depth_idx = find(data.depth > 0 & cut_idx);
    cut_idx(positive_depth_idx) = false(length(positive_depth_idx), 1);
%         
%     figure;
%     plot(data.time(cut_idx), data.des_twist_lin_x(cut_idx))
%     hold on;grid on;
%     plot(data.time(cut_idx), data.des_twist_lin_y(cut_idx))
%     plot(data.time(cut_idx), data.des_twist_lin_z(cut_idx))
%     plot(data.time(cut_idx), data.depth(cut_idx))
%     
%     figure;
%     plot(data.time, data.des_twist_lin_x)
%     hold on;grid on;
%     plot(data.time, data.des_twist_lin_y)
%     plot(data.time, data.des_twist_lin_z)
%     plot(data.time, data.depth)
%     plot(data.time, data.filt_ft_force_x/10)
    
%     figure;
%     plot(data.time(cut_idx), data.filt_ft_force_x(cut_idx))
%     hold on;grid on;
%     plot(data.time(cut_idx), filter(b,a,data.ft_force_y(cut_idx)))
%     plot(data.time(cut_idx), data.filt_ft_force_z(cut_idx))

    cut_time = data.time(cut_idx);
    cut_data = [data.time data.depth];
    twist = [data.ee_twist_lin_x data.ee_twist_lin_y, data.ee_twist_lin_z];
    cut_data = [cut_data filter(b, a, twist)];
    cut_data = [cut_data data.filt_ft_force_x filter(b,a,data.ft_force_y) data.filt_ft_force_z];
    cut_data(:,1) = cut_data(:,1) - cut_data(1,1);
    straight_cuts{cut} = cut_data(cut_idx, :);
    cut = cut + 1;
end

if strcmp(fruit_name, 'orange') 
    straight_cuts(1) = [];
    straight_cuts(6) = [];
    straight_cuts(9) = [];
    straight_cuts(11) = [];
    straight_cuts(1) = [];
    straight_cuts(11) = [];
    straight_cuts(1) = [];
    straight_cuts(1) = [];
% elseif fruit_name == 'apple'
%     break
end
figure;
subplot(2,2,1);
hold on;grid on;
for cut=1:length(straight_cuts)
    data = straight_cuts{cut};
    plot(data(:,3), data(:,2));
end
ylabel('depth')
xlabel('twist X')
xlim([-0.05 0.04])
% xlim([-0.005 0.035])
subplot(2,2,3);
hold on;grid on;
for cut=1:length(straight_cuts)
    data = straight_cuts{cut};
    plot(data(:,4), data(:,2));
end
ylabel('depth')
xlabel('twist Y')
xlim([-0.05 0.04])
% xlim([-0.005 0.035])
subplot(2,2,2);
hold on;grid on;
for cut=1:length(straight_cuts)
    data = straight_cuts{cut};
    plot(data(:,6), data(:,2));
end
ylabel('depth')
xlabel('force X')
xlim([-10 3])
% xlim([-10 1])
subplot(2,2,4);
hold on;grid on;
for cut=1:length(straight_cuts)
    data = straight_cuts{cut};
    plot(data(:,7), data(:,2));
end
ylabel('depth')
xlabel('force Y')
xlim([-10 3])
% xlim([-10 1])


figure;
hold on;grid on;
for cut=1:length(straight_cuts)
    data = straight_cuts{cut};
    plot3(data(:,3), data(:,6), data(:,2));
end
xlabel('twist X')
ylabel('force X')
zlabel('depth')


figure;
hold on;grid on;
for cut=1:length(straight_cuts)
    data = straight_cuts{cut};
    plot3(data(:,4), data(:,7), data(:,2));
end
xlabel('twist Y')
ylabel('force Y')
zlabel('depth')
