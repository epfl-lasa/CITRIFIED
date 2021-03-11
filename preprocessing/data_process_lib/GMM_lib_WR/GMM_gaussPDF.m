function prob = gaussPDF(Data, Mu, Sigma)% in row 72 of GMM_EM
%
% This function computes the Probability Density Function (PDF) of a
% multivariate Gaussian represented by means and covariance matrix.
%
% Author:	Sylvain Calinon, 2009
%			http://programming-by-demonstration.org
%
% Inputs -----------------------------------------------------------------
%   o Data:  D x N array representing N datapoints of D dimensions.
%   o Mu:    D x K array representing the centers of the K GMM components.
%   o Sigma: D x D x K array representing the covariance matrices of the 
%            K GMM components.
% Outputs ----------------------------------------------------------------
%   o prob:  1 x N array representing the probabilities for the 
%            N datapoints.     

[nbVar,nbData] = size(Data);

Data = Data' - repmat(Mu',nbData,1);%explain: https://blog.csdn.net/anqier1009/article/details/5214978
           %Chinese:repmat jiang juzhen duidie zai yi qi, [mu' mu' mu'.....]', this row means calcute the distance of data and mu   
prob = sum((Data*inv(Sigma)).*Data, 2);
prob = exp(-0.5*prob) / sqrt((2*pi)^nbVar * (abs(det(Sigma))+realmin));
