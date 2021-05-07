clc; close all; clear all;

% data_path = '../../data/raw_data/march_esn_test/two_more_tws/';
% % dir_content = struct2cell(dir([data_path '*apple_02_*.json']'));
% dir_content = struct2cell(dir([data_path '*.json']'));
% average_time_between_predictions = [];
% average_time_covered_time_window = [];
% time_after_trigger = [];
% reception_times = [];
% covered_times = [];
% 
% for nb_file=1:size(dir_content, 2)
%     nb_file
% %     esns = 1;
% %     esn_messages = [];
%     reception_time = [];
%     covered_time = [];
%     triggered = 0;
%     first = 0;
%     file = fopen([data_path dir_content{1,nb_file}]);
%     jsondecode(fgetl(file));
%     while ~feof(file)
%         message = jsondecode(fgetl(file));
%         if strcmp(message.control.phase,'insertion') && ~triggered
%             triggered = message.time;
%         end
%         if isfield(message, 'esn')
% %             esn_messages{esns} = message.esn;
% %             esns = esns + 1;
%             reception_time = [reception_time; message.time];
%             covered_time = [covered_time; message.esn.input.time(end) - message.esn.input.time(1)];
%             if ~first
%                 first = 1;
%                 time_after_trigger = [time_after_trigger; message.time - triggered];
%             end
%         end
%     end
%     reception_times = [reception_times; reception_time - reception_time(1)];
%     covered_times = [covered_times; covered_time];
%     average_time_between_predictions = [average_time_between_predictions; mean(diff(reception_time))];
%     average_time_covered_time_window = [average_time_covered_time_window; mean(covered_time)];
% end
load('timing_analysis_600.mat');

figure;
plot(average_time_between_predictions, 'b', 'LineWidth', 1.5)
hold on; grid on;
plot([1 length(average_time_between_predictions)], [mean(average_time_between_predictions) mean(average_time_between_predictions)], 'r')
plot(average_time_covered_time_window, 'g', 'LineWidth', 1.5)
plot([1 length(average_time_covered_time_window)], [mean(average_time_covered_time_window) mean(average_time_covered_time_window)], 'k')
plot(time_after_trigger)
plot([1 length(time_after_trigger)], [mean(time_after_trigger) mean(time_after_trigger)], 'k')
xlabel('trial')
ylabel('time [s]')
legend('average time between predicitons within trials', ...
       ['average time between predicitons across trials = ' num2str(mean(average_time_between_predictions)) ', std = ' num2str(std(average_time_between_predictions))], ...
       'average time window length within trials', ...
       ['average time window length across trials = ' num2str(mean(average_time_covered_time_window)) ', std = ' num2str(std(average_time_covered_time_window))], ...
       'time to first prediction', ...
       ['average time to first prediction = ' num2str(mean(time_after_trigger)) ', std = ' num2str(std(time_after_trigger))]);
   

figure;
hold on; grid on;
load('timing_analysis_900.mat');
plot(average_time_between_predictions,'LineWidth', 1.5)
plot([1 length(average_time_between_predictions)], [mean(average_time_between_predictions) mean(average_time_between_predictions)])
legend_s{1} = 'average time between predicitons within trials with 900';
legend_s{2} = ['average time between predicitons across trials = ' num2str(mean(average_time_between_predictions)) ', std = ' num2str(std(average_time_between_predictions))];
load('timing_analysis_600.mat');
plot(average_time_between_predictions, 'LineWidth', 1.5)
plot([1 length(average_time_between_predictions)], [mean(average_time_between_predictions) mean(average_time_between_predictions)])
legend_s{3} = 'average time between predicitons within trials with 600';
legend_s{4} = ['average time between predicitons across trials = ' num2str(mean(average_time_between_predictions)) ', std = ' num2str(std(average_time_between_predictions))];
load('timing_analysis_300.mat');
plot(average_time_between_predictions, 'LineWidth', 1.5)
plot([1 length(average_time_between_predictions)], [mean(average_time_between_predictions) mean(average_time_between_predictions)])
legend_s{5} = 'average time between predicitons within trials with 300';
legend_s{6} = ['average time between predicitons across trials = ' num2str(mean(average_time_between_predictions)) ', std = ' num2str(std(average_time_between_predictions))];
xlim([1 5])
xlabel('trial')
ylabel('time [s]')
legend(legend_s{:});

   
% figure;
% hold on; grid on;
% load('timing_analysis_900.mat');
% plot(average_time_covered_time_window,'LineWidth', 1.5)
% plot([1 length(average_time_covered_time_window)], [mean(average_time_covered_time_window) mean(average_time_covered_time_window)])
% load('timing_analysis_600.mat');
% plot(average_time_covered_time_window, 'LineWidth', 1.5)
% plot([1 length(average_time_covered_time_window)], [mean(average_time_covered_time_window) mean(average_time_covered_time_window)])
% load('timing_analysis_300.mat');
% plot(average_time_covered_time_window, 'LineWidth', 1.5)
% plot([1 length(average_time_covered_time_window)], [mean(average_time_covered_time_window) mean(average_time_covered_time_window)])
% xlim([1 5])
% xlabel('trial')
% ylabel('time [s]')
% legend('average time window length within trials with 900', ...
%        ['average time window length across trials = ' num2str(mean(average_time_covered_time_window)) ', std = ' num2str(std(average_time_covered_time_window))], ...
%        'average time window length within trials with 600', ...
%        ['average time window length across trials = ' num2str(mean(average_time_covered_time_window)) ', std = ' num2str(std(average_time_covered_time_window))], ...
%        'average time window length within trials with 300', ...
%        ['average time window length across trials = ' num2str(mean(average_time_covered_time_window)) ', std = ' num2str(std(average_time_covered_time_window))]);
% 
