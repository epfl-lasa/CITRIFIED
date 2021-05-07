function [ out ] = majorityvote_tie(in, probabilities, labels)
% instead of majorityvote([label1, label2, label3]), run this additionally 
% with a 3x3 matrix containing the probability distribution for each timewindow, 
% where one row corresponds to one timewindow and the columns to the class
% indices, and a vector 1x3 containing the labels for your classes (for example [2,7,9]).
% for example: majorityvote([label1,label2,label3], all_output_test(1:3,:), [2,7,9])

[count,values]=hist(in,unique(in));
[Vmax,argmax]=max(count);
if Vmax < length(in) / 2
    sum_probabilities = sum(probabilities,1);
    [~, argmax] = max(sum_probabilities);
    out = labels(argmax);
else
    out=values(argmax);
end
end