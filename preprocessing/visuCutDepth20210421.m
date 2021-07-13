clear all; close all; clc

%% loop through each file 
straight_cuts = {};
cut = 1;

data_path = '../data/raw_data/april/depth_controlled_test/';
dir_content = struct2cell(dir([data_path '*apple*cut*.json']'));

for nb_file=1:size(dir_content, 2)
    nb_file
    time = [];
    force = [];
    velocity = [];
    position = [];
    depth = [];
    depth_offset = [];
    depth_time = [];
    file = fopen([data_path dir_content{1,nb_file}]);
    message = jsondecode(fgetl(file));
    while ~feof(file)
        message = jsondecode(fgetl(file));
        if isfield(message,'control') && strcmp(message.control.phase,'cut') && isfield(message, 'model')
            for b = message.filtered.bodies'
                if strcmp(b{1}.frame, 'robot') && strcmp(b{1}.name, 'ee')
                    velocity = [velocity; b{1}.twist.linear(1)];
                    position = [position; b{1}.pose.position'];
                end
                if strcmp(b{1}.frame,'robot') && strcmp(b{1}.name,'ft_sensor')
                    force = [force; b{1}.wrench.force(1)];
                end
                if strcmp(b{1}.frame, 'robot') && strcmp(b{1}.name, 'task')
                    depth_offset = b{1}.pose.position(3);
                end
            end
            for b = message.raw.bodies'
                if strcmp(b{1}.frame, 'robot') && strcmp(b{1}.name, 'task')
                    depth_offset = b{1}.pose.position(3);
                end
            end
            time = [time; message.time];
            depth = [depth; message.model.depth];
%             depth = [depth; message.model.depth + depth_offset];
        end
    end
    fclose(file);
    data = [time depth velocity force];
    data(:,1) = data(:,1) - data(1,1);
    data(position(:,1)>0.54,:) = [];
    straight_cuts{cut} = data;
    cut = cut + 1;
end

%%    
transient = 0.25;
figure;
sgtitle('Oranges')
ax1 = subplot(3,1,1);
hold on; grid on;
for cut=1:length(straight_cuts)
    data = straight_cuts{cut};
    data(data(:,1)<transient,:) = [];
    plot(data(:,1), data(:,2));
end
xlabel('time')
ylabel('depth')

ax2 = subplot(3,1,2);
hold on; grid on;
for cut=1:length(straight_cuts)
    data = straight_cuts{cut};
    data(data(:,1)<transient,:) = [];
    plot(data(:,1), data(:,3));
end
xlabel('time')
ylabel('velocity X')

ax3 = subplot(3,1,3);
hold on; grid on;
for cut=1:length(straight_cuts)
    data = straight_cuts{cut};
    data(data(:,1)<transient,:) = [];
    plot(data(:,1), data(:,4));
end
xlabel('time')
ylabel('force X')
linkaxes([ax1,ax2,ax3],'x')

figure;
hold on;grid on;
for cut=1:length(straight_cuts)
    data = straight_cuts{cut};
    data(data(:,1)<transient,:) = [];
    plot3(data(:,2), data(:,3), data(:,4), 'r');
end
load 'apple_straight_controlled.mat'
for cut=1:length(straight_cuts)
    data = straight_cuts{cut};
    data(data(:,1)<transient,:) = [];
    plot3(data(:,2), data(:,3), data(:,4), 'b');
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
    data(data(:,1)<transient,:) = [];
    plot(data(:,1), data(:,2));
end
xlabel('time')
ylabel('depth')

ax2 = subplot(3,1,2);
hold on; grid on;
for cut=1:length(straight_cuts)
    data = straight_cuts{cut};
    data(data(:,1)<transient,:) = [];
    plot(data(:,1), data(:,3));
end
xlabel('time')
ylabel('velocity X')

ax3 = subplot(3,1,3);
hold on; grid on;
for cut=1:length(straight_cuts)
    data = straight_cuts{cut};
    data(data(:,1)<transient,:) = [];
    plot(data(:,1), data(:,4));
end
xlabel('time')
ylabel('force X')
linkaxes([ax1,ax2,ax3],'x')
