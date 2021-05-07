function [prediction_distribution, predicted_indices, scores, errors] = classify_and_evaluate(real_labels, predicted_output, nb_splits)

prediction_distribution = [];
predicted_indices = [];
success_rate = 0;
decision_gap = [];
max_conf = [];
errors = zeros(1,length(predicted_output));
nb_outputs = size(predicted_output{1},2);

avg_predicted_output = [];
for i = 1:length(predicted_output)
    split = floor(size(predicted_output{i}) / nb_splits);
    avg_predicted_output{i} = zeros(nb_splits, nb_outputs);
    for j=1:nb_splits-1
        avg_predicted_output{i}(j,:) = mean(predicted_output{i}((j-1)*split+1:j*split,:));
    end
    avg_predicted_output{i}(nb_splits,:) = mean(predicted_output{i}((nb_splits-1)*split+1:end, :));
    sum_avg_predicted_output = sum(avg_predicted_output{i});
    softmax_sum = sum(arrayfun(@(x) exp(x),sum_avg_predicted_output));
    if isinf(softmax_sum)
        [~, max_index] = max(sum_avg_predicted_output);
        softmax_predicted_output = zeros(1,nb_outputs);
        softmax_predicted_output(max_index) = 1;
    else
        softmax_predicted_output = arrayfun(@(x) exp(x),sum_avg_predicted_output)./softmax_sum;
    end
    prediction_distribution = [prediction_distribution;  softmax_predicted_output];

    [~, ind] = max(softmax_predicted_output);
    predicted_indices(end+1) = ind;

    if real_labels(i) == ind
        success_rate = success_rate + 1;
        sorted = sort(softmax_predicted_output);
        decision_gap(end+1) = sorted(end) - sorted(end-1);
        max_conf(end+1) = max(softmax_predicted_output);
    else
        errors(i) = ind;
    end
    
%     normalized_predicted_output = sum_avg_predicted_output / sum(sum_avg_predicted_output);
%     prediction_distribution = [prediction_distribution;  normalized_predicted_output];
% 
%     [~, ind] = max(normalized_predicted_output);
%     predicted_indices(end+1) = ind;
% 
%     if real_labels(i) == ind
%         success_rate = success_rate + 1;
%         sorted = sort(normalized_predicted_output);
%         decision_gap(end+1) = sorted(end) - sorted(end-1);
%         max_conf(end+1) = max(normalized_predicted_output);
%     else
%         errors(i) = ind;
%     end     
end

scores.success_rate  = 100 * (success_rate / length(predicted_output));
scores.avg_decision_gap  = mean(decision_gap);
scores.std_decision_gap  = std(decision_gap);
scores.avg_max_conf  = mean(max_conf);
scores.std_max_conf  = std(max_conf);

end