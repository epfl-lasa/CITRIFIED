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
    straight_cuts{cut} = data;
end

%% with X values
figure;
hold on;grid on;
for cut=1:length(straight_cuts)
    data = straight_cuts{cut};
    plot3(data(:,6), data(:,9), data(:,2));
end
xlabel('twist X')
ylabel('force X')
zlabel('depth')

figure;
hold on;grid on;
for cut=1:length(straight_cuts)
    data = straight_cuts{cut};
    depth_derivative = diff(data(:,2))/mean(diff(data(:,1)));
    depth_derivative = [depth_derivative(1); depth_derivative];
    plot3(data(:,6), depth_derivative, data(:,2));
end
xlabel('twist X')
ylabel('depth derivative')
zlabel('depth')

figure;
hold on;grid on;
for cut=1:length(straight_cuts)
    data = straight_cuts{cut};
    depth_derivative = diff(data(:,2))/mean(diff(data(:,1)));
    depth_derivative = [depth_derivative(1); depth_derivative];
    plot3(data(:,9), depth_derivative, data(:,2));
end
xlabel('force X')
ylabel('depth derivative')
zlabel('depth')


figure;
hold on;grid on;
for cut=1:length(straight_cuts)
    data = straight_cuts{cut};
    depth_derivative = diff(data(:,2))/mean(diff(data(:,1)));
    depth_derivative = [depth_derivative(1); depth_derivative];
    plot3(data(:,6), data(:,9), depth_derivative);
end
xlabel('twist X')
ylabel('force X')
zlabel('depth derivative')

%% with Z values
figure;
hold on;grid on;
for cut=1:length(straight_cuts)
    data = straight_cuts{cut};
    plot3(data(:,8), data(:,11), data(:,2));
end
xlabel('twist Z')
ylabel('force Z')
zlabel('depth')

figure;
hold on;grid on;
for cut=1:length(straight_cuts)
    data = straight_cuts{cut};
    depth_derivative = diff(data(:,2))/mean(diff(data(:,1)));
    depth_derivative = [depth_derivative(1); depth_derivative];
    plot3(data(:,8), depth_derivative, data(:,2));
end
xlabel('twist Z')
ylabel('depth derivative')
zlabel('depth')

figure;
hold on;grid on;
for cut=1:length(straight_cuts)
    data = straight_cuts{cut};
    depth_derivative = diff(data(:,2))/mean(diff(data(:,1)));
    depth_derivative = [depth_derivative(1); depth_derivative];
    plot3(data(:,11), depth_derivative, data(:,2));
end
xlabel('force Z')
ylabel('depth derivative')
zlabel('depth')


figure;
hold on;grid on;
for cut=1:length(straight_cuts)
    data = straight_cuts{cut};
    depth_derivative = diff(data(:,2))/mean(diff(data(:,1)));
    depth_derivative = [depth_derivative(1); depth_derivative];
    plot3(data(:,8), data(:,11), depth_derivative);
end
xlabel('twist Z')
ylabel('force Z')
zlabel('depth derivative')
