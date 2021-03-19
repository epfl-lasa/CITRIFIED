function [prediction_distribution, scores, errors] = classify_and_evaluate(output, predicted_output, nb_splits)

prediction_distribution = [];
success_rate = 0;
max_conf = [];
errors = zeros(1,length(predicted_output));

for i = 1:length(predicted_output)
    time_window = floor(size(predicted_output{i}) / nb_splits);
    avg_predicted_output{i} = zeros(nb_splits , size(output{i}, 2));
    for j=1:nb_splits-1
        avg_predicted_output{i}(j,:) = mean(predicted_output{i}((j-1)*time_window+1:j*time_window,:));
    end
    avg_predicted_output{i}(nb_splits,:) = mean(predicted_output{i}((nb_splits-1)*time_window+1:end, :));
    sum_avg_predicted_output = sum(avg_predicted_output{i});
    normalized_predicted_output = sum(avg_predicted_output{i}) / sum(sum(avg_predicted_output{i}));
    prediction_distribution = [prediction_distribution;  normalized_predicted_output];

    [~, ind] = max(normalized_predicted_output);
    predicted_class = zeros(1,size(output{i}, 2));
    predicted_class(ind) = 1;

    if sum(predicted_class == output{i}(1,:)) == size(output{i}, 2)
        success_rate = success_rate + 1;
        max_conf = [max_conf; max(normalized_predicted_output)];
    else
        errors(i) = ind;
    end     
end

scores.success_rate  = 100 * (success_rate / length(predicted_output));
scores.avg_max_conf  = mean(max_conf);
scores.std_max_conf  = std(max_conf);

end