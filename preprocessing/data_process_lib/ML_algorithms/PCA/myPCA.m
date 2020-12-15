function [W,U]=myPCA(data,stndrd)

% this function returns the transformation matrix W and the eigenvalues U
% after applying Principal Component Analysis (PCA) in the data. Both W and
% U are sorted by increasing order with respect to the eigenvalues.
%
%               Xt=W'*data
%
% Inputs:
%   
%   data:   a matrix containing the original data. The matrix data should
%           be organized collumn-wise, meaning that the collumns should 
%           correspond to the dimensions (variables-features) while the 
%           rows corresponds to the observations.
%           
%   stndrd: an option indicating if the data should be standardized. if
%           the value is 'standardization' the data will be standardized
%
% Outputs:
%   
%   W:  the transformation matrix containing the eigenvectors collumn-wise
%   U:  a vector containing the eigenvalues in a descending order
%
%

% figure(2)
% plot(data(:,1),data(:,2),'b*')
% xlabel('1^s^t dimension')
% ylabel('2^s^t dimension')
% grid on
%title('the first two dimensions of the original data')


% standardize the data:

sdata=data-repmat(mean(data),size(data,1),1);


% standardize the data according to the option
if strcmp(stndrd,'standardization')
    for i=1:size(sdata,2)
    
        sdata(:,i)=sdata(:,i)/sqrt(cov_matrix(data(:,i)));
    
    end
end


% find the covariance matrix

cov_m=cov_matrix(sdata);

% find the eigenvalues and the eigenvectors
[V,D]=eig(cov_m);

% sorting the eigenvalues in descending order and the eigenvectors with
% respect to the order of the eigenvalues 
[U,order]=sort(diag(D),'descend');

W=V(:,order);

end