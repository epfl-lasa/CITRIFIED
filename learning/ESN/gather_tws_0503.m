clear all; close all; clc

%% ESN classes
classes = {'apple','banana','orange','prune'};
colors = {'g','y','r','b'};

%% data paths
raw_data_path = fullfile('..','..','data','raw_data','mai');

%% load file by file
% each insertion log file is loaded, the three time windows are extracted
% and put into a cell array of size 1x3 which in turn is put into a big
% cell array of size #classes x #trials per class

% all_timewindows = [];
% for nb_class = 1:length(classes)
%     fprintf('\n----- %sS -----\n',upper(classes{nb_class}));
%     files = dir([raw_data_path,'/*',classes{nb_class},'*.json']);
%     files_ok = 0;
%     for file = files'
%         esn_messages = [];
%         file_id = fopen(fullfile(file.folder, file.name));
%         while ~feof(file_id)
%             message = jsondecode(fgetl(file_id));
%             if isfield(message, 'esn') && isfield(message.esn,'input')
%                 message.esn.real_class_name = classes{nb_class};
%                 message.esn.real_class_index = nb_class - 1;
%                 esn_messages{end+1} = message.esn;
%             end
%         end
%         if length(esn_messages) ~= 3
%             fprintf('Not exactly three time windows in this file, skipping it: \n%s\n\n', file.name)
%             continue
%         else
%             files_ok = files_ok + 1;
%         end
%         all_timewindows{nb_class, files_ok} = esn_messages;
%     end
% end
% save(fullfile(raw_data_path,'timewindows_20210503.mat'),'all_timewindows');
load(fullfile(raw_data_path,'timewindows_20210503.mat'));

%% 2x2 3D plot of velocity X, depth and force derivative X
figure;
for nb_class = 1:size(all_timewindows,1)
    ax = subplot(2,2,nb_class);
    title(classes{nb_class});
    hold on; grid on;
    data = all_timewindows(nb_class,:);
    data = data(~cellfun(@isempty, data));
    for insertion = 1:length(data)
        for tw = 1:3
            input = data{insertion}{tw}.input;
            plot3(input.depth, input.velocity_z, input.force_z, colors{nb_class});
        end
    end
    xlabel('depth')
    ylabel('velocity z')
    zlabel('force z')
    view(ax,[30 25]);
end

figure;
hold on; grid on;
for nb_class = 1:size(all_timewindows,1)
    data = all_timewindows(nb_class,:);
    data = data(~cellfun(@isempty, data));
    for insertion = 1:length(data)
        for tw = 1:3
            input = data{insertion}{tw}.input;
            plot3(input.depth, input.velocity_z, input.force_z, colors{nb_class});
        end
    end
end
xlabel('depth')
ylabel('velocity z')
zlabel('force derivative z')
view(ax,[30 25]);

figure;
title('apples - green, oranges - red')
hold on; grid on;
for nb_class = 1:size(all_timewindows,1)
    data = all_timewindows(nb_class,:);
    data = data(~cellfun(@isempty, data));
    for insertion = 1:length(data)
        for tw = 1:3
            input = data{insertion}{tw}.input;
            plot3(input.depth, input.velocity_z, input.force_z, colors{nb_class});
        end
    end
end
xlabel('depth [m]')
ylabel('velocity z [m/s]')
zlabel('force z [N]')
view(ax,[30 25]);

figure('Position',[100 100 800 600]);
hold on; grid on;
for nb_class = 1:1
    data = all_timewindows(nb_class,:);
    data = data(~cellfun(@isempty, data));
    for insertion = 1:length(data)
        first = 0;
        for tw = 1:3
            input = data{insertion}{tw}.input;
            if first == 0
                first = input.time(1);
            end
            subplot(3,1,1);hold on;
            plot(input.time-first, 1e3*input.depth,'r')
            ylabel('Depth [m]')
            ylim([0 8])
            set(gca,'FontSize',13)
            subplot(3,1,2);hold on;
            plot(input.time-first, input.force_z,'g')
            ylabel('Force Z [N]')
            ylim([-4 0])
            set(gca,'FontSize',13)
            subplot(3,1,3);hold on;
            plot(input.time-first, input.velocity_z,'b')
            ylabel('Velocity Z [m/s]')
            ylim([-0.04 0])
            xlabel('Time [s]')
            set(gca,'FontSize',13)
        end
    end
end
    