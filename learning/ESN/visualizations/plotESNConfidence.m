clear all; close all; clc

%% loop through each file

classes = {'apple', 'banana', 'orange', 'prune'};
% for class=classes
%     data_path = '../data/raw_data/april/esn_finish_test/';
%     dir_content = struct2cell(dir([data_path '*' class{1} '*.json']'));
%     insertions = [];
% 
%     for nb_file=1:size(dir_content, 2)
%         nb_file
%         time = [];
%         esn_messages = [];
%         file = fopen([data_path dir_content{1,nb_file}]);
%     %     fruitName = extractFruitName(dir_content{1,nb_file});
%         message = jsondecode(fgetl(file));
%         while ~feof(file)
%             message = jsondecode(fgetl(file));
%             if isfield(message,'esn')
%                 esn_messages{end+1} = message.esn;
%                 time(end+1) = message.time;
%             end
%         end
%         fclose(file);
%         if size(esn_messages) < 3
%             continue
%         end
%         insertions{end+1} = esn_messages;
%     end
%     save([class{1} '_esn_finish_test'], 'insertions');
% end

%%
colors = {'g','r','y','b'};
for real_class=classes
    load([real_class{1} '_esn_finish_test'], 'insertions');
    figure;
    hold on;
    title(real_class{1})
    for insertion=1:length(insertions)
%     for insertion=1:1
        start_time = insertions{insertion}{1}.input.time(1);
        for tw=1:3
            data = insertions{insertion}{tw};
            probabilities = softmax(data.probabilities);
%             for class=1:4
%                 plot([data.input.time(1) data.input.time(end)]-start_time,[probabilities(class) probabilities(class)],colors{class})
%             end
            if strcmp(real_class,data.class_name)
                plot([data.input.time(1) data.input.time(end)]-start_time,[probabilities(data.class_index+1) probabilities(data.class_index+1)],'g')
            else
                plot([data.input.time(1) data.input.time(end)]-start_time,[probabilities(data.class_index+1) probabilities(data.class_index+1)],'r')
            end
        end
    end
end

%%
function pred=softmax(A)
softmax_sum = sum(arrayfun(@(x) exp(x),A));
pred = arrayfun(@(x) exp(x),A)./softmax_sum;
end