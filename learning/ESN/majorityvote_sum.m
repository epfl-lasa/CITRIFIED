function [ out ] = majorityvote_sum(probabilities, labels)
% instead of majorityvote([label1, label2, label3]), run this with a 3x3
% matrix containing the probability distribution for each timewindow, where
% one row corresponds to one timewindow and the columns to the class
% indices, and a vector 1x3 containing the labels for your classes (for example [2,7,9]).
% for example majorityvote(all_output_test(1:3,:), [2,7,9])

sum_probabilities = sum(probabilities,1);

[~, argmax] = max(sum_probabilities);
out = labels(argmax);
end