function [Priors, Mu, Sigma] = EM_init_kmeans(Data, nbStates)
%
% This function initializes the parameters of a Gaussian Mixture Model 
% (GMM) by using k-means clustering algorithm.
%
% Author:	Sylvain Calinon, 2009
%			http://programming-by-demonstration.org
%
% Inputs -----------------------------------------------------------------
%   o Data:     D x N array representing N datapoints of D dimensions.
%   o nbStates: Number K of GMM components.
% Outputs ----------------------------------------------------------------
%   o Priors:   1 x K array representing the prior probabilities of the
%               K GMM components.
%   o Mu:       D x K array representing the centers of the K GMM components.
%   o Sigma:    D x D x K array representing the covariance matrices of the 
%               K GMM components.
% Comments ---------------------------------------------------------------
%   o This function uses the 'kmeans' function from the MATLAB Statistics 
%     toolbox. If you are using a version of the 'netlab' toolbox that also
%     uses a function named 'kmeans', please rename the netlab function to
%     'kmeans_netlab.m' to avoid conflicts. 

[nbVar, nbData] = size(Data);

%Use of the 'kmeans' function from the MATLAB Statistics toolbox
[Data_id, Centers] = kmeans(Data', nbStates); 
%the explain of this kmeans:  http://blog.sciencenet.cn/blog-41996-450513.html
%and  :    https://www.csdn.net/article/2012-07-03/2807073-k-means
Mu = Centers';
for i=1:nbStates
  idtmp = find(Data_id==i);%the explain of this find: http://blog.sina.com.cn/s/blog_707b64550100rbh3.html
  Priors(i) = length(idtmp);
  Sigma(:,:,i) = cov([Data(:,idtmp) Data(:,idtmp)]');%cov : https://blog.csdn.net/clam_clam/article/details/7335588
  %Add a tiny variance to avoid numerical instability
  Sigma(:,:,i) = Sigma(:,:,i) + 1E-5.*diag(ones(nbVar,1));
end
Priors = Priors ./ sum(Priors);

