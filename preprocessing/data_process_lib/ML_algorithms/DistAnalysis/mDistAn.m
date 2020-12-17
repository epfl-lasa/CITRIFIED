function model = mDistAn(data, labels, varargin)

% Function for performing discriminal analysis (Linear or Quadratic)
%
% Input: 
%   data:       a matrix of the data (number of samples x number of features. 
%               The data should be organized in a column-wise fashion,
%               meaning that each column should corespond to a feature (or
%               dimension) and each row should correspond to a sample (or
%               observation)
%   
%   label:      a vector containing the labels of the data. The length of
%               the vector should be equal to the number of samples of the
%               data (number of rows) and it should be an one-to-one 
%               correspondence (the 1st row of the data should correspond
%               to labels(1), the 2nd row of the data to labels(2), etc.)
%
%   optional inputs:
%   
%   Normalization:  [true or false] perfoming a normalization of the data
%                   with the maximum value (default: false)
%
%   Standarization: [true or false] performing a standarization of the
%                   data by extracting the mean values (default: false)
%
%   Type:           [LDA, QDA, QDA+] performing linear, quadratic 
%                   discriminant analysis or quadratic discriminant anlysis
%                   with GMM (default: QGA)
%
%   max_nb_components: [integer] the maximum number of Gaussian distributions
%                       in the GMM (default: 3)
%
% Output:
%   model:      a struct containing the discriminant model based on the
%               input data
%               

% check if the data have the same length
if ( size(data, 1) ~= length(labels))
    error("the number of samples should be equal to the number of labels");
end

model = struct();

% define default values
doNormalization = false;

doStandarization = false;

type = "QDA";

max_nb_components = 3;

% get the options from the user, if any
if (length(varargin) >= 2)
    for i=1:2:length(varargin)
        if(varargin{i} == "Normalization")
            doNormalization = logical(varargin{i + 1});
        end
        if(varargin{i} == "Standarization")
            doStandarization = logical(varargin{i + 1});
        end
        if(varargin{i} == "Type")
            type = varargin{i + 1};
        end
        if(varargin{i} == "max_nb_components")
            max_nb_components = varargin{i + 1};
        end
    end
end

% normalize and/or standatize the data if needed
model.normalization = doNormalization;
model.maxValues = max(data);
if (doNormalization)
    data = data ./ model.maxValues;
end

model.standarization = doStandarization;
model.overallMean = mean(data);
if (doStandarization)
    data = data - model.overallMean;
end

% find the labels and number of unique labels
uniqueLabels = unique(labels);
nbLabels = length(uniqueLabels);

% compute the within-class and between-classes scatters, (means and
% covariances - commented)
% class_means = [];
% class_covs = {};
SB = zeros(size(data,2), size(data,2));
SW = zeros(size(data,2), size(data,2));
for i=1:nbLabels
    class_data = data(labels == uniqueLabels(i), :);
    class_mean = mean(class_data);
    class_cov = (class_data - class_mean)' * (class_data - class_mean) / (size(class_data, 1) -1);
%     class_means = [class_means; class_mean];
%     class_covs{i} = class_cov;
    SW = SW + class_cov;
    
    SB = SB + ( (class_mean - model.overallMean)' * (class_mean - model.overallMean)) * size(class_data, 1);
end

% if LDA is selected, take the average covariance matrix as the
% between-class covariance. However, the user must check if the covariance
% is the same across the classes
if type == "LDA"
    SW = SW / nbLabels;
end

% compute the transformation matrix W
mm = SW \ SB;

[eigVec, eigVal] = eig(mm);

[~, idx] = sort(diag(eigVal), 'descend');

model.eigenValues = diag(eigVal(idx, idx));
model.eigenVEctors = eigVec(:, idx);
nbKeepVec = length(model.eigenValues(model.eigenValues > 0.0001));
model.W = eigVec(:, 1:nbKeepVec);

% project data to the new hyperplane
proj_data = data * model.W;

% plot the data for inspection
figure
scatter(proj_data(labels == 1,1), proj_data(labels == 1,2))
hold on
scatter(proj_data(labels == 2,1), proj_data(labels == 2,2))
scatter(proj_data(labels == 3,1), proj_data(labels == 3,2))
title("after transformation")
legend('class 1', 'class 3', 'class 3')

% create a probabilistic model for each class
prob_model_mu = {};
prob_model_sigma = {};
gmmModel = {};
for i=1:nbLabels
    % get the projected data for the corresponding label
    proj_class_data = proj_data(labels == uniqueLabels(i), :);
    
    % compute the new mean and covariance matrix
    prob_model_mu{i} = mean(proj_class_data);
    prob_model_sigma{i} = (proj_class_data - prob_model_mu{i})' * (proj_class_data - prob_model_mu{i}) / (size(proj_class_data, 1) - 1);
    
    % if the QDA+ option is selected, check if the GMM model could have 
    % more than one components and fit a GMM to the data. Else, the GMM
    % model would have only one components, fitting only one Gaussian 
    % distribution to the data
    if type == "QDA+"
        tmp_model = {};
        AIC_val = zeros(max_nb_components, 1);
        for j=1:max_nb_components
            tmp_model{j} = fitgmdist(proj_class_data, j);
            % using the Akaike information criterion (AIC), to select the
            % number of components of the GMM
            AIC_val(j) = tmp_model{j}.AIC; 
        end
        % find the best model based according to AIC
        [~,idx_model] = min(AIC_val);
        % keep the winner model
        gmmModel{i} = tmp_model{idx_model};
    else
        % fit one Gaussian distribution to the data
        gmmModel{i} = fitgmdist(proj_class_data, 1);
    end
    
end

model.mu = prob_model_mu;
model.Sigma = prob_model_sigma;
model.gmmModel = gmmModel;

end