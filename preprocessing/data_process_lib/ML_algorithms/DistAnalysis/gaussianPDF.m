function [likelihood] = gaussianPDF(data, mu, sigma)
% Function for computing the likelihood from a Gaussian distribution
%
% Input: 
%   data:       a matrix of the data (number of samples x number of features. 
%               The data should be organized in a column-wise fashion,
%               meaning that each column should corespond to a feature (or
%               dimension) and each row should correspond to a sample (or
%               observation)
%   
%   mu:         a vector containing the mean of the Gaussian distribution o
%
%      
%   sigma:      the covariance matrix of the Gaussian distribution
%
% Output:
%   likelihood: the likelihood of each sample on the Gaussian distribution. 
%               There is an one-to-one correspondence with the rows of the
%               data (the first likelihod corresponds to the first row of 
%               the data, the second label to the second row of the data, 
%               etc.)
%
    
% get the dimensions of the data
dim = length(mu);       

% check if the input arguments have the proper dimension 
if (size(data, 2) ~= dim) && (size(data, 1) ~= dim)
    error("the dimensions of data should be equal to the dimension of mu");
end
if size(sigma, 1) ~= dim
    error("the dimensions of sigma should be equal to the dimension of mu");
end

% compute the determinant of the covariance
dSigma = det(sigma);
if dSigma == 0
    % if the determinant is zero the covariance is not inversable
	error('the covariance matrix is singular')
else
    % compute the eigenvalues of the covariance matrix
    % for checking that the covariance matrix is positive definite
	[~, eigVal] = eig(sigma);
	eigVal = diag(eigVal);
	nb_zeros = length( eigVal(eigVal == 0) );
	nb_negatives = length( eigVal(eigVal < 0) );
	if nb_negatives ~= 0
        error("the covariance matrix is not positive definite");
    end
	if nb_zeros ~= 0
        error("the covariance matrix is positive semi-definite");
    end
    
    % if all the checks are ok, compute the likelihood for each sample
	likelihood = zeros(size(data,1), 1);
    for i = 1:size(data,1)
        likelihood(i) = exp(-(1/2) * (data(i,:) - mu) * sigma * (data(i,:) - mu)')/sqrt( ((2*pi)^(dim)) * dSigma );
	end
end

end