clear all; close all; clc

%% loop through each file 
straight_cuts = {};
cut = 1;

data_path = '../data/raw_data/mai/';
dir_content = struct2cell(dir([data_path '*apple*cut*.json']'));

% for nb_file=1:size(dir_content, 2)
%     nb_file
%     time = [];
%     force = [];
%     velocity = [];
%     position = [];
%     depth = [];
%     ee = [];
%     file = fopen([data_path dir_content{1,nb_file}]);
%     message = jsondecode(fgetl(file));
%     while ~feof(file)
%         message = jsondecode(fgetl(file));
%         if isfield(message,'control') && strcmp(message.control.phase,'cut') && isfield(message, 'model')
%             for b = message.filtered.bodies'
%                 if strcmp(b{1}.frame, 'robot') && strcmp(b{1}.name, 'ee')
%                     velocity = [velocity; b{1}.twist.linear(1)];
%                     position = [position; b{1}.pose.position'];
%                 end
%                 if strcmp(b{1}.frame,'robot') && strcmp(b{1}.name,'ft_sensor')
%                     force = [force; b{1}.wrench.force(1)];
%                 end
%             end
%             for b = message.raw.bodies'
%                 if strcmp(b{1}.frame, 'task') && strcmp(b{1}.name, 'ee')
%                     ee(end+1) = b{1}.pose.position(3);
%                 end
%             end
%             time = [time; message.time];
%             depth = [depth; message.model.depth];
%         end
%     end
%     fclose(file);
%     data = [time position(:,1) depth velocity force ee'];
%     data(:,1) = data(:,1) - data(1,1);
%     data(:,2) = data(:,2) - data(1,2);
%     straight_cuts{cut} = data;
%     cut = cut + 1;
% end
% save(fullfile(data_path,'apple_straight_0503.mat'),'straight_cuts');


%%    
x_column = 2;
x_label = 'cut distance';
transient = 0.00;
% 
% freq = 9.380037948039710e+02; % this is the sampling frequency of the raw data from february
% [b, a] = butter(4, 6/freq*2); % create a Butterworth 1D filter with cutoff frequency 6Hz
%                               % 6/freq*2 is the normalized cutoff frequency


figure;
load(fullfile(data_path,'orange_straight_0503.mat'));
sgtitle('Oranges')
ax1 = subplot(3,1,1);
hold on; grid on;
for cut=1:length(straight_cuts)
    data = straight_cuts{cut};
    data(data(:,x_column)<transient,:) = [];
    plot(data(:,x_column), data(:,3));
end
xlabel(x_label)
ylabel('depth')

ax2 = subplot(3,1,2);
hold on; grid on;
for cut=1:length(straight_cuts)
    data = straight_cuts{cut};
    data(data(:,x_column)<transient,:) = [];
%     plot(data(:,x_column), data(:,3) + data(:,end))
    plot(data(:,x_column), data(:,4));
end
xlabel(x_label)
ylabel('velocity X')
% ylabel('estimated height')

ax3 = subplot(3,1,3);
hold on; grid on;
for cut=1:length(straight_cuts)
    data = straight_cuts{cut};
    data(data(:,x_column)<transient,:) = [];
    plot(data(:,x_column), data(:,5));
end
xlabel(x_label)
ylabel('force X')
% ylabel('ee in task')
linkaxes([ax1,ax2,ax3],'x')

figure;
hold on;grid on;
for cut=1:length(straight_cuts)
    data = straight_cuts{cut};
    data(data(:,x_column)<transient,:) = [];
    plot3(data(:,3), data(:,4), data(:,5), 'r');
end

load(fullfile(data_path,'apple_straight_0503.mat'));

for cut=1:length(straight_cuts)
    data = straight_cuts{cut};
    data(data(:,x_column)<transient,:) = [];
    plot3(data(:,3), data(:,4), data(:,5), 'g');
end
xlabel('depth')
ylabel('velocity X')
zlabel('force X')


figure;
sgtitle('Apples')
ax1 = subplot(3,1,1);
hold on; grid on;
for cut=1:length(straight_cuts)
    data = straight_cuts{cut};
    data(data(:,x_column)<transient,:) = [];
    plot(data(:,x_column), data(:,3));
end
xlabel(x_label)
ylabel('depth')

ax2 = subplot(3,1,2);
hold on; grid on;
for cut=1:length(straight_cuts)
    data = straight_cuts{cut};
    data(data(:,x_column)<transient,:) = [];
    plot(data(:,x_column), data(:,4));
end
xlabel(x_label)
ylabel('velocity X')

ax3 = subplot(3,1,3);
hold on; grid on;
for cut=1:length(straight_cuts)
    data = straight_cuts{cut};
    data(data(:,x_column)<transient,:) = [];
    plot(data(:,x_column), data(:,5));
end
xlabel(x_label)
ylabel('force X')
linkaxes([ax1,ax2,ax3],'x')
