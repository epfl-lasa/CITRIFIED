clear all; close all; clc

%% ESN classes
classes = {'apple','banana','orange','prune'};
colors = {'g','y','r','b'};

%% data paths
raw_data_path = fullfile('..','..','data','raw_data','mai','test');

%% load file by file
% each insertion log file is loaded, the three time windows are extracted
% and put into a cell array of size 1x3 which in turn is put into a big
% cell array of size #classes x #trials per class
% 
% all_timewindows = [];
% for nb_class = 1:length(classes)
%     fprintf('\n----- %sS -----\n',upper(classes{nb_class}));
%     files = dir([raw_data_path,'/*0510*',classes{nb_class},'*.json']);
%     files_ok = 0;
%     for file = files'
%         esn_messages = [];
%         file_id = fopen(fullfile(file.folder, file.name));
%         message = jsondecode(fgetl(file_id));
%         while ~feof(file_id)
%             message = jsondecode(fgetl(file_id));
%             if isfield(message, 'esn')
%                 message.esn.real_class_name = classes{nb_class};
%                 message.esn.real_class_index = nb_class - 1;
%                 esn_messages{end+1} = message.esn;
%             end
%         end
% %         if length(esn_messages) ~= 3
% %             fprintf('Not exactly three time windows in this file, skipping it: \n%s\n\n', file.name)
% %             continue
% %         else
%             files_ok = files_ok + 1;
% %         end
%         all_timewindows{nb_class, files_ok} = esn_messages;
%     end
% end
% save(fullfile(raw_data_path,'timewindows_test_20210510.mat'),'all_timewindows');
load(fullfile(raw_data_path,'timewindows_test_20210510.mat'));

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
xlabel('depth [m]')
ylabel('velocity z [m/s]')
zlabel('force z [N]')
view(ax,[30 25]);

%%
final_classification = {};
wrong = 0;
data = all_timewindows(~cellfun(@isempty, all_timewindows));
for i=1:length(data)
    final_classification{end+1} = data{i}{4};
    if ~strcmp(final_classification{end}.class_name,final_classification{end}.real_class_name)
        wrong = wrong + 1;
        final_classification{end}.class_name
    end
end
wrong / length(data)
    