
addpath(genpath('D:\matlab2017\work\10_ML_toolbox'))

close all; clear all; clc;

variance=0.5;
[Xs,Ys]=meshgrid(linspace(-5,5,100),linspace(-5,5,100));
X=[Xs(:),Ys(:)];

rbf=exp(-sum(X.^2,2)/variance^2);

contour(Xs,Ys,reshape(rbf,100,100),10)
