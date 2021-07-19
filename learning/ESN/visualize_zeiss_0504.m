close all; clear all; clc;
addpath(genpath(fullfile('.','echo-state-networks','lib','ESNToolbox')));
raw_data_path = fullfile('..','..','data','raw_data','mai');
colors = {'g','y','r','b'};

%% Confusion matrix
load 'mat_files/best_result.mat';
classes = {'Apple','Banana','Orange','Prune'};

figure;
% C = confusionmat([best_result.train_labels best_result.test_labels], [best_result.predicted_train_labels best_result.predicted_test_labels]);
C = confusionmat(best_result.test_labels, best_result.predicted_test_labels);
C_norm = 100 * C ./ sum(C,2);
imagesc(1:1:length(classes),1:1:length(classes),C_norm, [0 100]);
colorbar;
map = [linspace(0.95,0,100)', linspace(0.95,0,100)', linspace(0.95,256/256,100)'];
colormap(map)
xticks(1:1:length(classes))
yticks(1:1:length(classes))
xticklabels(classes)
yticklabels(classes)
ylabel('Real class')
xlabel('Predicted class')
for i=1:length(classes)
    for j=1:length(classes)
        text(i,j,num2str(round(C_norm(j,i),2)),'HorizontalAlignment','center','FontSize',16)
    end
end
set(gca,'FontSize',18)
clearvars -except raw_data_path colors

%% Raw data in 3D subplots
load(fullfile(raw_data_path,'timewindows_20210503.mat'));
classes = {'Apples','Bananas','Oranges','Prunes'};
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
            plot3(1e3*input.depth, input.velocity_z, input.force_derivative_z, colors{nb_class});
        end
    end
    xlabel('depth [mm]')
    ylabel('velocity z [m/s]')
    zlabel('force derivative z [N]')
    view(ax,[30 25]);
    set(gca,'FontSize',14)
end
clearvars -except raw_data_path colors

%% Bar plots ESN three time windows
load 'mat_files/best_result.mat';
load(fullfile(raw_data_path,'timewindows_20210503.mat'));
classes = {'Apple','Banana','Orange','Prune'};
esn = best_result.trained_esn;
tws = all_timewindows{1,3};
predicted_test_output = [];
depths = [];
times = [];

test_input = [];
test_output_labels = [];
for j = 1:length(tws)
    input = tws{j}.input;
    test_input{end+1} = [input.depth, input.velocity_x, input.velocity_z ...
                          input.force_x, input.force_z, ...
                          input.force_derivative_x, input.force_derivative_z];
    test_output_labels(end+1) = tws{j}.real_class_index + 1;
    depths = [depths input.depth];
    times = [times input.time];
end
for j = 1:length(tws)
    predicted_test_output{j} = test_esn(test_input{j}, esn, 10);
end
clear j output

[distribution, ~, ~, ~] = classify_and_evaluate(test_output_labels, predicted_test_output, 3);

red_map = [linspace(0.95,1,100)', linspace(0.95,0,100)', linspace(0.95,0,100)'];
green_map = [linspace(0.95,0,100)', linspace(0.95,1,100)', linspace(0.95,0,100)'];
blue_map = [linspace(0.95,0,100)', linspace(0.95,0,100)', linspace(0.95,1,100)'];

figure('Position',[100 100 800 600]);
title('First time window')
hold on; grid on;
for i = 1:length(classes)
    prob = round(distribution(1,i),2);
    h = bar(i,prob);
%     set(h,'FaceColor',colors{i});
    set(h,'FaceColor',red_map(floor(prob*100),:));
    xticks(1:1:length(classes))
    xticklabels(classes)
    ylim([0 2])
    set(gca,'FontSize',14)
end
ylabel('Cumultative probability distribtuion')

figure('Position',[100 100 800 600]);
hold on; grid on;
title('First and second time window')
for i = 1:length(classes)
    prob = round(distribution(1:2,i),2);
    h = bar(i,prob,'stacked');
%     set(h,'FaceColor',colors{i});
    h(1).FaceColor = red_map(floor(prob(1,:)*100),:);
    h(2).FaceColor = green_map(floor(prob(2,:)*100),:);
    xticks(1:1:length(classes))
    xticklabels(classes)
    ylim([0 2])
    set(gca,'FontSize',14)
end
ylabel('Cumultative probability distribtution')

figure('Position',[100 100 800 600]);
hold on; grid on;
title('All three time windows')
for i = 1:length(classes)
    prob = round(distribution(1:3,i),2);
    h = bar(i,prob,'stacked');
%     set(h,'FaceColor',colors{i});
    h(1).FaceColor = red_map(floor(prob(1,:)*100),:);
    h(2).FaceColor = green_map(floor(prob(2,:)*100),:);
    h(3).FaceColor = blue_map(floor(prob(3,:)*100),:);
    xticks(1:1:length(classes))
    xticklabels(classes)
    ylim([0 2])
    set(gca,'FontSize',14)
end
ylabel('Cumultative probability distribtution')

figure('Position',[100 100 800 600]);
hold on; grid on;
title('Final classification')
for i = 1:length(classes)
    prob = round(mean(distribution(1:3,i)),2);
    h = bar(i,prob);
%     set(h,'FaceColor',colors{i});
    set(h,'FaceColor',green_map(floor(prob*100),:));
    xticks(1:1:length(classes))
    xticklabels(classes)
%     ylim([0 2])
    set(gca,'FontSize',14)
end
ylabel('Mean probability distribution')

%% Animated time line video with time windows
start = times(1,1);

figure('Position',[100 100 1500 160]);
box off;
first_tw = animatedline('LineWidth',15,'Color','r');
first_received = animatedline('Marker','*','Color', 'r', 'MarkerSize',10);
second_tw = animatedline('LineWidth',15,'Color', 'g');
second_received = animatedline('Marker','*','Color', 'g', 'MarkerSize',10);
third_tw = animatedline('LineWidth',15,'Color', 'b');
third_received = animatedline('Marker','*','Color', 'b', 'MarkerSize',10);
first = false;
second = false;
third = false;

time = unique(reshape(times,[],1));
time = [time; (time(end):mean(diff(time)):time(end)+0.05)'];
xlim([0 times(end,end)-start+0.05])
yticks([])
xlabel('Time [s]')
ylim([0.5 2.5])
a = tic; % start timer
k = 1;
set(gca,'FontSize',15)

% myVideo = VideoWriter('myVideoFile'); %open video file
% myVideo.FrameRate = 50;  %can adjust this, 5 - 10 works well for me
% open(myVideo)
while k <= size(time,1)
    
    b = toc(a); % check timer
    if b > (1/30)
        if time(k) >= times(1,1) && time(k) <= times(end,1)
            addpoints(first_tw,time(k)-start,1)
        end
        if time(k) >= times(end,1)+0.045 && ~first
            addpoints(first_received,times(end,1)+0.045-start,1)
            first = true;
        end
        
        if time(k) >= times(1,2) && time(k) <= times(end,2)
            addpoints(second_tw,time(k)-start,1.5)
        end
        if time(k) >= times(end,2)+0.045 && ~second
            addpoints(second_received,times(end,2)+0.045-start,1.5)
            second = true;
        end
        
       
        if time(k) >= times(1,3) && time(k) <= times(end,3)
            addpoints(third_tw,time(k)-start,2)
        end
        if time(k) >= times(end,3)+0.045 && ~third
            addpoints(third_received,times(end,3)+0.045-start,2)
            third = true;
        end
        drawnow % update screen every 1/30 seconds
        a = tic; % reset timer after updating
        k = k+1;
    end
%     frame = getframe(gcf); %get frame
%     writeVideo(myVideo, frame);
end
% close(myVideo)
clearvars -except raw_data_path colors

%% Time series animation of insertion data
load 'mat_files/best_result.mat';
load(fullfile(raw_data_path,'timewindows_20210503.mat'));
classes = {'Apple','Banana','Orange','Prune'};
esn = best_result.trained_esn;
tws = all_timewindows{1,3};
depths = [];
times = [];
force_z = [];
velocity_z = [];
for j = 1:length(tws)
    input = tws{j}.input;
    force_z = unique([force_z;input.force_z]);
    velocity_z = unique([velocity_z;input.velocity_z]);
    depths = unique([depths;input.depth]);
    times = unique([times;input.time]);
end

figure('Position',[100 100 1000 800]);

sub1 = subplot(3,1,1);
depth_line = animatedline('LineWidth',2,'Color','r');
ylabel('Depth [m]')
ylim([0 7])
xlim([0 times(end)-times(1)])
% xlabel('Time [s]')
set(gca,'FontSize',15)

sub2 = subplot(3,1,2);
force_line = animatedline('LineWidth',2,'Color', 'g');
ylabel('Force Z [N]')
ylim([-4 0])
xlim([0 times(end)-times(1)])
% xlabel('Time [s]')
set(gca,'FontSize',15)

sub3 = subplot(3,1,3);
velocity_line = animatedline('LineWidth',2,'Color', 'b');
ylabel('Velocity Z [m/s]')
ylim([-0.04 0])
xlim([0 times(end)-times(1)])
xlabel('Time [s]')
set(gca,'FontSize',15)
a = tic; % start timer
k = 1;

% myVideo = VideoWriter('timeseries'); %open video file
% myVideo.FrameRate = 50;  %can adjust this, 5 - 10 works well for me
% open(myVideo)
times = times - times(1);
while k <= size(times,1)
    b = toc(a); % check timer
    if b > (1/30)
        addpoints(depth_line,times(k),1e3*depths(k));
        addpoints(force_line,times(k),force_z(k));
        addpoints(velocity_line,times(k),velocity_z(k));
        drawnow % update screen every 1/30 seconds
        a = tic; % reset timer after updating
        k = k+1;
    end
%     frame = getframe(gcf); %get frame
%     writeVideo(myVideo, frame);
end
% close(myVideo)
clearvars -except raw_data_path colors

%% Boxplots of probability distribution
load 'mat_files/best_result.mat';
load(fullfile(raw_data_path,'timewindows_20210503.mat'));
esn = best_result.trained_esn;
classes = {'Apple','Banana','Orange','Prune'};
figure('Position',[100,100,1200,900]);
sgtitle('Classifier condfidence (train + test)','FontSize',16,'FontWeight','bold')
for i=1:length(classes)
    predicted_test_output = [];
    test_input = [];
    test_output_labels = [];
    data = all_timewindows(i,:);
    data = data(~cellfun(@isempty, data));
    for j = 1:length(data)
        for tw = 1:3
            input = data{j}{tw}.input;
            test_input{end+1} = [input.depth, input.velocity_x, input.velocity_z ...
                                 input.force_x, input.force_z, ...
                                 input.force_derivative_x, input.force_derivative_z];
            test_output_labels(end+1) = data{j}{tw}.real_class_index + 1;
        end
    end
    for j = 1:length(test_input)
        predicted_test_output{j} = test_esn(test_input{j}, esn, 10);
    end
    [distribution, ~, ~, ~] = classify_and_evaluate(test_output_labels, predicted_test_output, 3);
    subplot(2,2,i)
    title(classes(i))
    boxplot(distribution)
    xticklabels(classes)
    ylim([0 1])
    set(gca,'FontSize',14)
end
