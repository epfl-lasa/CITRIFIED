clear all; close all; clc

%% loop through each file 
straight_cuts = {};
cut = 1;

data_path = '../data/raw_data/april/scalpel_test/';
dir_content = struct2cell(dir([data_path '*orange*cut*.json']'));

% for nb_file=1:size(dir_content, 2)
%     nb_file
%     time = [];
%     force = [];
%     velocity = [];
%     position = [];
%     depth = [];
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
%                 if strcmp(b{1}.frame, 'robot') && strcmp(b{1}.name, 'task')
%                     depth_offset = b{1}.pose.position(3);
%                 end
%             end
%             time = [time; message.time];
%             depth = [depth; message.model.depth];
%         end
%     end
%     fclose(file);
%     data = [time position(:,1) depth velocity force];
%     data(:,1) = data(:,1) - data(1,1);
%     data(:,2) = data(:,2) - data(1,2);
%     straight_cuts{cut} = data;
%     cut = cut + 1;
% end
% save(fullfile(data_path,'orange_straight_cuts.mat'),'straight_cuts');


%%    
x_column = 2;
x_label = 'cut distance';
transient = 0.009;

figure;
load(fullfile(data_path,'orange_straight_cuts.mat'));
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

figure;
hold on;grid on;
for cut=1:length(straight_cuts)
    data = straight_cuts{cut};
    data(data(:,x_column)<transient,:) = [];
    plot3(data(:,3), data(:,4), data(:,5), 'r');
end

load(fullfile(data_path,'apple_straight_cuts.mat'));

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
