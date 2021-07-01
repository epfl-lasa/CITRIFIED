clear all; close all; clc

%% data paths
raw_data_path = fullfile('..','..','data','raw_data','mai','test');

%% load file by file
% each insertion log file is loaded, the three time windows are extracted
% and put into a cell array of size 1x3 which in turn is put into a big
% cell array of size #classes x #trials per class
 
dir_content = struct2cell(dir([raw_data_path '/*0510*silicon*.json']'));
insertion = 1;
for nb_file=2:size(dir_content, 2)
    nb_file
    time = [];
    force = [];
    velocity = [];
    position = [];
    depth = [];
    file = fopen(fullfile(raw_data_path, dir_content{1,nb_file}));
    message = jsondecode(fgetl(file));
    while ~feof(file)
        message = jsondecode(fgetl(file));
        if isfield(message,'control') && strcmp(message.control.phase,'insertion')
            for b = message.filtered.bodies'
                if strcmp(b{1}.frame, 'robot') && strcmp(b{1}.name, 'ee')
                    velocity = [velocity; b{1}.twist.linear(3)];
                    position = [position; b{1}.pose.position'];
                end
                if strcmp(b{1}.frame,'robot') && strcmp(b{1}.name,'ft_sensor')
                    force = [force; b{1}.wrench.force(3)];
                end
            end
            time = [time; message.time];
%             depth = [depth; message.model.depth];
        end
    end
    fclose(file);
    data = [time position(:,3) velocity force];
    data(:,1) = data(:,1) - data(1,1);
    data(:,2) = data(:,2) - data(1,2);
    insertions{insertion} = data;
    insertion = insertion + 1;
end
% save(fullfile(data_path,'apple_straight_0503.mat'),'straight_cuts');
% load(fullfile(raw_data_path,'timewindows_test_20210510.mat'));

%%
figure;
hold on;
plot(-data(:,2),data(:,4))
