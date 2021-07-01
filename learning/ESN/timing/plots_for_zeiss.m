clc; close all; clear all;

esn_nine = {};
esn_six = {};
esn_four = {};
esns = 1;

figure(1)
title('Time windows')
data_path = '../../../data/raw_data/march_esn_test/';
dir_content = struct2cell(dir([data_path '*.json']'));
file = fopen([data_path dir_content{1,1}]);
jsondecode(fgetl(file));
triggered = 0;
while ~feof(file)
    message = jsondecode(fgetl(file));
    if strcmp(message.control.phase,'insertion') && ~triggered
        triggered = message.time;
    end
    if isfield(message, 'esn')
        esn_nine{esns} = message.esn;
        esns = esns + 1;
    end
end
subplot(3,1,1)
hold on;
for i=1:length(esn_nine)
    msg = esn_nine{i};
    if i > 1
        plot(msg.input.time(end)-triggered, i+0.5, 'k*')
    end
    plot([msg.input.time(1)-triggered msg.input.time(end)-triggered], [i i], 'r', 'LineWidth', 10)
end
xlim([0 1])
ylim([0 i+1])
yticks([0:1:i+1])
ylabel('900 Internal Units', 'FontSize', 12)

esns = 1;
data_path = '../../../data/raw_data/march_esn_test/comparison/';
dir_content = struct2cell(dir([data_path '*0401_apple_01*.json']'));
file = fopen([data_path dir_content{1,1}]);
jsondecode(fgetl(file));
triggered = 0;
while ~feof(file)
    message = jsondecode(fgetl(file));
    if strcmp(message.control.phase,'insertion') && ~triggered
        triggered = message.time;
    end
    if isfield(message, 'esn')
        esn_six{esns} = message.esn;
        esns = esns + 1;
    end
end
subplot(3,1,2)
hold on;
for i=1:length(esn_six)
    msg = esn_six{i};
    if i > 1
        plot(msg.input.time(end)-triggered, i+0.5, 'k*')
    end
    plot([msg.input.time(1)-triggered msg.input.time(end)-triggered], [i i], 'b', 'LineWidth', 10)
end
xlim([0 1])
ylim([0 i+1])
yticks([0:1:i+1])
ylabel('600 Internal Units', 'FontSize', 12)

esns = 1;
data_path = '../../../data/raw_data/march_esn_test/comparison/';
dir_content = struct2cell(dir([data_path '*0406*.json']'));
file = fopen([data_path dir_content{1,1}]);
jsondecode(fgetl(file));
triggered = 0;
while ~feof(file)
    message = jsondecode(fgetl(file));
    if strcmp(message.control.phase,'insertion') && ~triggered
        triggered = message.time;
    end
    if isfield(message, 'esn')
        esn_four{esns} = message.esn;
        esns = esns + 1;
    end
end
delta_t = esn_four{2}.input.time(1) - esn_four{1}.input.time(1);
duration = esn_four{1}.input.time(end) - esn_four{1}.input.time(1);
subplot(3,1,3)
hold on;
for i=1:length(esn_four)
    msg = esn_four{i};
    if i > 1
        plot(msg.input.time(end)-triggered, i+0.5, 'k*')
    end
    plot([msg.input.time(1)-triggered msg.input.time(end)-triggered], [i i], 'g', 'LineWidth', 10)
end
for i=3:5
    msg = esn_four{2};
    plot((i-2)*delta_t + msg.input.time(end)-triggered, i+0.5, 'k*')
    plot([(i-2)*delta_t + msg.input.time(end)-triggered-duration (i-2)*delta_t + msg.input.time(end)-triggered], [i i], 'g', 'LineWidth', 10)
end
xlim([0 1])
ylim([0 i+1])
yticks([0:1:i+1])
xlabel('Time [s]')
ylabel('400 Internal Units', 'FontSize', 12)
legend('Time window', 'Reception of prediction', 'FontSize', 12)

load('../mat_files/timing_analysis_300')
x = [300];
y = [mean(average_time_between_predictions)];
load('../mat_files/timing_analysis_400')
x = [x 400];
y = [y mean(average_time_between_predictions)];
load('../mat_files/timing_analysis_600')
x = [x 600];
y = [y mean(average_time_between_predictions)];
load('../mat_files/timing_analysis_900')
x = [x 900];
y = [y mean(average_time_between_predictions)];
figure(2)
hold on; grid on;
plot(x(1):1:x(end), 1e3*polyval(polyfit(x,y,2),x(1):1:x(end)),'--k','LineWidth',2)
plot(x, 1e3*y,'r*','MarkerSize',10,'LineWidth',2)
xlabel('Number of internal units','FontSize',12)
ylabel('Computation time for prediction [ms]','FontSize',12)
legend('Polynomial function of order 2','Recorded computation times','Location','NorthWest','FontSize',12)


