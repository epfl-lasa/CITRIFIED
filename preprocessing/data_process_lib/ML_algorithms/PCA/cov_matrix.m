function [CM]=cov_matrix(data)

% this function returns the covariance matrix of the data. If the input is
% a vector, the function returns the variance of the elements of the vector
%
% Input:
%   
%   data: a matrix containing the data. The function assumes that the
%         data are arranged collumn-wise, meaning that each collumn
%         corresponds to the observations of one variable (dimension)
%
% Output:
% 
%   CM: the covariance matrix of the input data. 
%   
%
% Example: let's assume that we have 3 variables (x,y,z) to our problem. We
%          observed n values for the variables so that:
%
%          X=[x1 x2 x3 ... xn]'
%          Y=[y1 y2 y3 ... yn]'
%          Z=[z1 z2 z3 ... zn]'
%
%          Then the input matrix will be: data=[X Y Z]
%
%          The output matrix will be:
%       
%                      cov(X,X)  cov(X,Y)  cov(X,Z)
%       CM=Cov(data)=  cov(Y,X)  cov(Y,Y)  cov(Y,Z)
%                      cox(Z,X)  cov(Z,Y)  cov(Z,Z)                   



% [observations,dimensions]=size(data);
% 
% CM=zeros(dimensions,dimensions);
% 
% for i=1:dimensions
%     for j=1:dimensions
%         CM(i,j)=(data(:,i)-mean(data(:,i)))'*(data(:,j)-mean(data(:,j)))/(observations-1);
%     end
% end

CM=((data-repmat(mean(data),size(data,1),1))'*(data-repmat(mean(data),size(data,1),1)))/(size(data,1)-1);


end