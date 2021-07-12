clear all; close all; clc

load('straight_cuts_orange');
% straight_cuts{3} = {};
straight_cuts = straight_cuts(~cellfun('isempty',straight_cuts));

freq = 9.380037948039710e+02; % this is the sampling frequency of the raw data from february
[b, a] = butter(4, 6/freq*2); % create a Butterworth 1D filter with cutoff frequency 6Hz
                              % 6/freq*2 is the normalized cutoff frequency
                            
%% discard first 300ms
for cut=1:length(straight_cuts)
    data = straight_cuts{cut};
    data(:,2) = filtfilt(b,a,data(:,2));
    data(:,6) = filtfilt(b,a,data(:,6));
    data(:,8) = filtfilt(b,a,data(:,8));
    data(:,1) = data(:,1) - data(1,1);
    data(data(:,1) < 0.3,:) = [];
    straight_cuts{cut} = data;
end

all_data = [];
for cut=1:length(straight_cuts)
    all_data = [all_data; straight_cuts{cut}];
end
% all_data(:,[1 3 4 5 7 10]) = [];

%% GPR
X_train = all_data(:,[2 6]);
y_train = all_data(:,9);

[coeff, score] = pca(X_train);
X_centered = score*coeff';

biplot(coeff(:,1:2),'scores',score(:,1:2),'varlabels',{'v_1','v_2'});