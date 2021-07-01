clear all; close all; clc

%% data paths
raw_data_path = fullfile('..','..','data','raw_data','mai','test');

%% load file by file
% each insertion log file is loaded, the three time windows are extracted
% and put into a cell array of size 1x3 which in turn is put into a big
% cell array of size #classes x #trials per class
 
% dir_content = struct2cell(dir([raw_data_path '/*0512*silicon*.json']'));
% insertion = 1;
% for nb_file=1:size(dir_content, 2)
%     if contains(dir_content{1,nb_file}, 'silicon_04')
%         continue
%     end
%     nb_file
%     time = [];
%     force = [];
%     velocity = [];
%     position = [];
%     depth = [];
%     file = fopen(fullfile(raw_data_path, dir_content{1,nb_file}));
%     message = jsondecode(fgetl(file));
%     while ~feof(file)
%         message = jsondecode(fgetl(file));
%         if isfield(message,'control') && strcmp(message.control.phase,'insertion')
%             for b = message.filtered.bodies'
%                 if strcmp(b{1}.frame, 'robot') && strcmp(b{1}.name, 'ee')
%                     velocity = [velocity; b{1}.twist.linear(3)];
%                     position = [position; b{1}.pose.position'];
%                 end
%                 if strcmp(b{1}.frame,'robot') && strcmp(b{1}.name,'ft_sensor')
%                     force = [force; b{1}.wrench.force(3)];
%                 end
%             end
%             time = [time; message.time];
% %             depth = [depth; message.model.depth];
%         end
%     end
%     fclose(file);
%     data = [time position(:,3) velocity force];
%     data(:,1) = data(:,1) - data(1,1);
%     data(:,2) = data(:,2) - data(1,2);
%     insertions{insertion} = data;
%     insertion = insertion + 1;
% end
% save(fullfile(raw_data_path,'silicon_0512.mat'),'insertions');
load(fullfile(raw_data_path,'silicon_0512.mat'));


%%
figure;
ax1 = subplot(4,1,1);
hold on;grid on;
for i = 2:5
    data = insertions{i};
    plot(-data(:,1),data(:,3))
end
ylabel('velocity')
ax2 = subplot(4,1,2);
hold on;grid on;
for i = 2:5
    data = insertions{i};
    plot(-data(2:end,1),diff(data(:,3)))
end
ylabel('acceleration')
ax3 = subplot(4,1,3);
hold on;grid on;
for i = 2:5
    data = insertions{i};
    plot(-data(:,1),data(:,4))
end
ylabel('force')
ax4 = subplot(4,1,4);
hold on;grid on;
for i = 2:5
    data = insertions{i};
    plot(-data(2:end,1),diff(data(:,4)))
end
ylabel('force derivative')
linkaxes([ax1 ax2 ax3 ax4], 'x')

%%
data = insertions{2};
data(find(data(:,1)>1.5,1):end,:) = [];
data(:,end+1) = data(:,1) > 0.8;
labeled{1} = data;

data = insertions{3};
data(find(data(:,1)>1.52,1):end,:) = [];
data(:,end+1) = data(:,1) > 0.89;
labeled{2} = data;

data = insertions{4};
data(find(data(:,1)>2.1,1):end,:) = [];
data(:,end+1) = data(:,1) > 1.6;
labeled{3} = data;

data = insertions{5};
data(find(data(:,1)>13.7,1):end,:) = [];
data(:,end+1) = data(:,1) > 13.2;
data(data(:,1)<12,:) = [];
labeled{4} = data;

save(fullfile(raw_data_path,'silicon_0512_labeled.mat'),'labeled');
%%
figure;
x_col = 1;
ax1 = subplot(4,1,1);
hold on;grid on;
for i = 1:length(labeled)
    data = labeled{i};
    plot(-data(:,x_col),data(:,3))
end
ylabel('velocity')
ax2 = subplot(4,1,2);
hold on;grid on;
for i = 1:length(labeled)
    data = labeled{i};
    plot(-data(2:end,x_col),diff(data(:,3)))
end
ylabel('acceleration')
ax3 = subplot(4,1,3);
hold on;grid on;
for i = 1:length(labeled)
    data = labeled{i};
    plot(-data(:,x_col),data(:,4))
end
ylabel('force')
ax4 = subplot(4,1,4);
hold on;grid on;
for i = 1:length(labeled)
    data = labeled{i};
    plot(-data(2:end,x_col),diff(data(:,4)))
end
ylabel('force derivative')
linkaxes([ax1 ax2 ax3 ax4], 'x')