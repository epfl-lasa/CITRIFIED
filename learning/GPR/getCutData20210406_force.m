clear all; close all; clc

load('straight_cuts_apple');
straight_cuts{3} = {};
straight_cuts = straight_cuts(~cellfun('isempty',straight_cuts));

freq = 9.380037948039710e+02; % this is the sampling frequency of the raw data from february
[b, a] = butter(4, 6/freq*2); % create a Butterworth 1D filter with cutoff frequency 6Hz
                              % 6/freq*2 is the normalized cutoff frequency
                              
straight_cuts_orange = load('straight_cuts_orange').straight_cuts;
                            
%% discard first 30ms
for cut=1:length(straight_cuts)
    data = straight_cuts{cut};
    data(:,2) = filtfilt(b,a,data(:,2));
    data(:,6) = filtfilt(b,a,data(:,6));
    data(:,8) = filtfilt(b,a,data(:,8));
    data(:,1) = data(:,1) - data(1,1);
    data(data(:,1) < 0.3,:) = [];
    straight_cuts{cut} = data;
end
for cut=1:length(straight_cuts_orange)
    data = straight_cuts_orange{cut};
    data(:,2) = filtfilt(b,a,data(:,2));
    data(:,6) = filtfilt(b,a,data(:,6));
    data(:,8) = filtfilt(b,a,data(:,8));
    data(:,1) = data(:,1) - data(1,1);
    data(data(:,1) < 0.3,:) = [];
    straight_cuts_orange{cut} = data;
end

%% velocity over time to ignore transient                         
% figure;
% hold on;grid on;
% for cut=1:length(straight_cuts)
%     data = straight_cuts{cut};
%     plot(data(:,1)-data(1,1), data(:,6));
% end
% xlabel('time')
% ylabel('velocity X')

%% force over time                         
% figure;
% hold on;grid on;
% for cut=1:length(straight_cuts)
%     data = straight_cuts{cut};
%     plot(data(:,1)-data(1,1), data(:,9));
% end
% xlabel('time')
% ylabel('force X')

%% with X values
figure;
hold on;grid on;
for cut=1:length(straight_cuts)
    data = straight_cuts{cut};
    plot3(-10*data(:,2), data(:,6), data(:,9), 'b');
end
for cut=1:length(straight_cuts_orange)
    data = straight_cuts_orange{cut};
    plot3(-10*data(:,2), data(:,6), data(:,9), 'r');
end
xlabel('depth')
ylabel('twist X')
zlabel('force X')

% figure;
% hold on;grid on;
% for cut=1:length(straight_cuts)
%     data = straight_cuts{cut};
%     dxdt = diff(data(:,6))/mean(diff(data(:,1)));
%     plot3([dxdt(1); dxdt], data(:,6), data(:,9));
% end
% xlabel('twist derivative X')
% ylabel('twist X')
% zlabel('force X')


%% with Z values
figure;
hold on;grid on;
for cut=1:length(straight_cuts)
    data = straight_cuts{cut};
    plot3(data(:,2), data(:,8), data(:,11));
end
xlabel('depth')
ylabel('twist Z')
zlabel('force Z')
