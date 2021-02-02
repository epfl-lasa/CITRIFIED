function [predictedLabel, likelihood, confidence] = mPredict(data, model)
% Function for predicting the label of the data according to the model
%
% Input: 
%   data:       a matrix of the data (number of samples x number of features. 
%               The data should be organized in a column-wise fashion,
%               meaning that each column should corespond to a feature (or
%               dimension) and each row should correspond to a sample (or
%               observation)
%   
%   model:      the model returned from the function mDistAn
%
% Output:
%   predictedLabel: the predicted labels. There is an one-to-one
%                   correspondence with the rows of the data (the first
%                   label corresponds to the first row of the data, the
%                   second label to the second row of the data, etc.)
%
%   likelihood:     the likelihood of the correspoding predicted label. 
%                   There is an one-to-one correspondence with the rows of 
%                   the data.
%
%   confidence:     a confidence metric for the predicted label. The
%                   confidence is defined as complement of the percentage 
%                   of the accumulated likelihood of the loosing classes
%                   over the likelihood of the winner class. There is an 
%                   one-to-one correspondence with the rows of the data.
%            

% normalize and/or standatize the data if needed
if (model.normalization)
    data = data ./ model.maxValues;
end


if (model.standarization)
    data = data - model.overallMean;
end

% project the data to the new hyper-plane
proj_data = data * model.W;

% compute the likelihood of each sample for each class
lik = zeros(length(model.gmmModel), size(data,1));

for i=1:length(model.gmmModel)
    lik(i,:) = pdf(model.gmmModel{i}, proj_data)';
    % if there is only one mixture component in the model, the function
    % gaussianPDF can be used too
end

% compute the likelihood of each sample for each class
[likelihood, predictedLabel] = max(lik);

likelihood = likelihood';
predictedLabel = predictedLabel';

% compute the confidence of each prediction
confidence = zeros(size(likelihood,2), 1);
for i=1:size(likelihood,2)
    tmp = lik(:,i);
    tmp(predictedLabel(i)) = [];
    confidence(i) = 1 - (sum(tmp)/likelihood(i));
end


end