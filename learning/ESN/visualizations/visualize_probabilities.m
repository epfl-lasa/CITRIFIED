clear all;close all;clc;

%% choice data set
data_of_exp = 'march_esn_test';
path_of_load = ['../../data/raw_data/' data_of_exp '/'];

classNames = {'apple','banana','orange','prune'};

%% loop through files
% dir_content = struct2cell(dir([path_of_load '*.json']'));
% 
% results = [];
% 
% for nb_file=1:size(dir_content, 2)
%     nb_file
%     dir_content{1,nb_file}
%     in = fopen([path_of_load dir_content{1,nb_file}], 'r');
%     message = jsondecode(fgetl(in));
%     
%     preds = {};
%     nb_preds = 1;
%     result = [];
%     name = split(message.metadata.trial, '_');
%     result.true_label = name{2};
%     
%     trigger_time = 0;
%     while ~feof(in)
%         message = jsondecode(fgetl(in));
%         if ~strcmp(message.control.phase, 'insertion') && ~strcmp(message.control.phase, 'pause')
%             continue
%         end
%         if ~trigger_time
%             trigger_time = message.time;
%         end
%         if isfield(message, 'esn')
%             result.pred_label = message.esn.class_name;
%             result.probabilities = softmax(message.esn.probabilities);
%             result.time = message.time - trigger_time;
%             result.depth = message.model.depth;
%             preds{nb_preds} = result;
%             nb_preds = nb_preds + 1;
%         end
%     end
%     results{nb_file} = preds;
%     fclose(in);
% end

%%
load('../mat_files/esn_results.mat')

for fruit=1:length(classNames)
    figure('Name',classNames{fruit});
    title(classNames{fruit});
    hold on; grid on;
    for i=1:length(results)
        result = results{i};
        if strcmp(result{1}.true_label,classNames{fruit})
            depth = [0 result{1}.depth];
            probs = zeros(4,2);
            for j=1:length(result)-1
                depth = [depth result{j}.depth result{j+1}.depth];
                probs = [probs result{j}.probabilities result{j}.probabilities];
            end
            depth = [depth 0.015];
            probs = [probs result{end}.probabilities];
            plot(1e3*depth(1:end-1), probs(1,1:end-1), 'g')
            plot(1e3*depth(1:end-1), probs(2,1:end-1), 'k')
            plot(1e3*depth(1:end-1), probs(3,1:end-1), 'b')
            plot(1e3*depth(1:end-1), probs(4,1:end-1), 'r')
        end
    end
    xlim([3 15])
    ylabel('probability')
    xlabel('depth [mm]')
    legend(classNames{:}, 'Location','NorthWest')
end
        

%%
function prob = softmax(p)
sum = 0;
for i=1:length(p)
    sum = sum + exp(p(i));
end
prob = p;
for i=1:length(p)
    prob(i) = exp(p(i)) / sum;
end
end

