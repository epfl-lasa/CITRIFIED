close all; clear all; clc;
addpath(genpath(fullfile('.','echo-state-networks','lib','ESNToolbox')));
raw_data_path = fullfile('..','..','data','raw_data','final');
colors = {'g','y','r','b'};

%% Bar plots ESN three time windows
file = fopen(fullfile(raw_data_path,'insertions', '20210525_banana_01_insertion_04.json'));
message = jsondecode(fgetl(file));
esn_messages = [];
time = [];
depths = [];
times = [];
force_z = [];
velocity_z = [];
distribution = [];
while ~feof(file)
    message = jsondecode(fgetl(file));
    if isfield(message,'esn') && isfield(message.esn,'input')
        esn_messages{end+1} = message.esn;
        distribution = [distribution; softmax(message.esn.probabilities')];
%         distribution = [distribution; (message.esn.probabilities')];
        times = [times message.esn.input.time];
        time(end+1) = message.time;
    end
end
% times(:,2) = mean(times(:,[1 3]),2);
fclose(file);

classes = {'Apple','Banana','Orange','Prune'};

red_map = [linspace(0.95,1,100)', linspace(0.95,0,100)', linspace(0.95,0,100)'];
green_map = [linspace(0.95,0,100)', linspace(0.95,1,100)', linspace(0.95,0,100)'];
blue_map = [linspace(0.95,0,100)', linspace(0.95,0,100)', linspace(0.95,1,100)'];

% figure('Position',[100 100 800 600]);
% title('First time window')
% hold on; grid on;
% for i = 1:length(classes)
%     prob = round(distribution(1,i),2);
%     h = bar(i,prob);
% %     set(h,'FaceColor',colors{i});
%     set(h,'FaceColor',red_map(floor(prob*100),:));
%     xticks(1:1:length(classes))
%     xticklabels(classes)
%     ylim([0 2])
%     set(gca,'FontSize',14)
% end
% ylabel('Cumultative probability distribtuion')
% 
% figure('Position',[100 100 800 600]);
% hold on; grid on;
% title('First and second time window')
% for i = 1:length(classes)
%     prob = round(distribution(1:2,i),2);
%     h = bar(i,prob,'stacked');
% %     set(h,'FaceColor',colors{i});
%     h(1).FaceColor = red_map(floor(prob(1,:)*100),:);
%     h(2).FaceColor = green_map(floor(prob(2,:)*100),:);
%     xticks(1:1:length(classes))
%     xticklabels(classes)
%     ylim([0 2])
%     set(gca,'FontSize',14)
% end
% ylabel('Cumultative probability distribtution')
% 
% figure('Position',[100 100 800 600]);
% hold on; grid on;
% title('All three time windows')
% for i = 1:length(classes)
%     prob = round(distribution(1:3,i),2);
%     h = bar(i,prob,'stacked');
% %     set(h,'FaceColor',colors{i});
%     h(1).FaceColor = red_map(floor(prob(1,:)*100),:);
%     h(2).FaceColor = green_map(floor(prob(2,:)*100),:);
%     h(3).FaceColor = blue_map(floor(prob(3,:)*100),:);
%     xticks(1:1:length(classes))
%     xticklabels(classes)
%     ylim([0 2])
%     set(gca,'FontSize',14)
% end
% ylabel('Cumultative probability distribtution')
% 
% figure('Position',[100 100 800 600]);
% hold on; grid on;
% title('Final classification')
% for i = 1:length(classes)
%     prob = round(mean(distribution(1:3,i)),2);
%     h = bar(i,prob);
% %     set(h,'FaceColor',colors{i});
%     set(h,'FaceColor',green_map(floor(prob*100),:));
%     xticks(1:1:length(classes))
%     xticklabels(classes)
% %     ylim([0 2])
%     set(gca,'FontSize',14)
% end
% ylabel('Mean probability distribution')

%% Animated time line video with time windows
% start = times(1,1);
% 
% figure('Position',[100 100 800 150]);
% box off;
% first_tw = animatedline('LineWidth',15,'Color','r');
% first_received = animatedline('Marker','*','Color', 'r', 'MarkerSize',10);
% second_tw = animatedline('LineWidth',15,'Color', 'g');
% second_received = animatedline('Marker','*','Color', 'g', 'MarkerSize',10);
% third_tw = animatedline('LineWidth',15,'Color', 'b');
% third_received = animatedline('Marker','*','Color', 'b', 'MarkerSize',10);
% first = false;
% second = false;
% third = false;
% 
% time = unique(reshape(times,[],1));
% time = [time; (time(end):mean(diff(time)):time(end)+0.05)'];
% xlim([0 times(end,end)-start+0.05])
% yticks([])
% xlabel('Time [s]')
% ylim([0.5 2.5])
% a = tic;
% k = 1;
% set(gca,'FontSize',15)
% 
% myVideo = VideoWriter('20210525_banana_01_insertion_04_timeline'); %open video file
% myVideo.FrameRate = 50;  %can adjust this, 5 - 10 works well for me
% open(myVideo)
% while k <= size(time,1)
%     
%     b = toc(a); % check timer
%     if b > (1/30)
%         if time(k) >= times(1,1) && time(k) <= times(end,1)
%             addpoints(first_tw,time(k)-start,1)
%         end
%         if time(k) >= times(end,1)+0.045 && ~first
%             addpoints(first_received,times(end,1)+0.045-start,1)
%             first = true;
%         end
%         
%         if time(k) >= times(1,2) && time(k) <= times(end,2)
%             addpoints(second_tw,time(k)-start,1.5)
%         end
%         if time(k) >= times(end,2)+0.045 && ~second
%             addpoints(second_received,times(end,2)+0.045-start,1.5)
%             second = true;
%         end
%         
%        
%         if time(k) >= times(1,3) && time(k) <= times(end,3)
%             addpoints(third_tw,time(k)-start,2)
%         end
%         if time(k) >= times(end,3)+0.045 && ~third
%             addpoints(third_received,times(end,3)+0.045-start,2)
%             third = true;
%         end
%         drawnow % update screen every 1/30 seconds
%         a = tic; % reset timer after updating
%         k = k+1;
%     end
%     frame = getframe(gcf); %get frame
%     writeVideo(myVideo, frame);
% end
% close(myVideo)

%% Time series animation of insertion data
file = fopen(fullfile(raw_data_path,'insertions', '20210525_apple_01_insertion_02.json'));
message = jsondecode(fgetl(file));
esn_messages = [];
time = [];
depths = [];
times = [];
force_z = [];
velocity_z = [];
distribution = [];
while ~feof(file)
    message = jsondecode(fgetl(file));
    if isfield(message,'esn') && isfield(message.esn,'input')
        esn_messages{end+1} = message.esn;
        times = unique([times; message.esn.input.time]);
        time(end+1) = message.time;
        force_z = unique([force_z;message.esn.input.force_z]);
        velocity_z = unique([velocity_z;message.esn.input.velocity_z]);
        depths = unique([depths;message.esn.input.depth]);
    end
end
fclose(file);
figure('Position',[100 100 800 900]);

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

% myVideo = VideoWriter('20210525_apple_01_insertion_02_data'); %open video file
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

%%
function pred=softmax(A)
softmax_sum = sum(arrayfun(@(x) exp(x),A));
pred = arrayfun(@(x) exp(x),A)./softmax_sum;
end