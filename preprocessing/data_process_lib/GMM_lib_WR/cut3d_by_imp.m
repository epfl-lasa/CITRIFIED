function [Data] = cut3d_by_imp(demos,time,imp)
%
% This function preprocess raw data and cut them in difference part as 
% format suitable for SEDS. The function cut the data of time and 
% trajectory based on the imp value, and the calculation is based on the 
% SEDS/ preprocess_demos. The function can be called using: 
%
%          [x0 , xT, Data, index] = cut3d_by_imp(demos,time,imp)
% Inputs -----------------------------------------------------------------
%
%
%   o demos:   A variable containing all demonstrations (only
%              trajectories). The variable 'demos' should follow the
%              following format:
%              - demos{n}: d x T^n matrix representing the d dimensional
%                          trajectories. T^n is the number of datapoint in
%                          this demonstration (1 < n < N)
%
%   o time:    This variable can be provided in two ways. If time steps
%              between all demonstrations are the same, 'time' could be
%              given as a positive scalar (i.e. time = dt). If not, 'time'
%              should follow the following format:
%              - time{n}: 1 x T^n vector representing the time array of length
%                         T^n corresponding to the first demo  (1 < n < N)
%
%
% Outputs ----------------------------------------------------------------
%
%   o x0:      d x 1 array representing the mean of all demonstration's
%              initial points.
%
%   o xT:      d x 1 array representing the mean of all demonstration's
%              final point (target point).
%
%   o Data:    A 2d x N_Total matrix containing all demonstration data points.
%              Rows 1:d corresponds to trajectories and the rows d+1:2d
%              are their first time derivatives. Each column of Data stands
%              for a datapoint. All demonstrations are put next to each other 
%              along the second dimension. For example, if we have 3 demos
%              D1, D2, and D3, then the matrix Data is:
%                               Data = [[D1] [D2] [D3]]
%
%   o index:   A vector of N+1 components defining the initial index of each
%              demonstration. For example, index = [1 T1 T2 T3] indicates
%              that columns 1:T1-1 belongs to the first demonstration,
%              T1:T2-1 -> 2nd demonstration, and T2:T3-1 -> 3rd
%              demonstration.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%    Copyright (c) 2010 S. Mohammad Khansari-Zadeh, LASA Lab, EPFL,   %%%
%%%          CH-1015 Lausanne, Switzerland, http://lasa.epfl.ch         %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% The program is free for non-commercial academic use. Please contact the
% author if you are interested in using the software for commercial purposes.
% The software must not be modified or distributed without prior permission
% of the authors. Please acknowledge the authors in any academic publications
% that have made use of this code or part of it. Please use this BibTex
% reference:
% 
% S. M. Khansari-Zadeh and A. Billard, "Learning Stable Non-Linear Dynamical 
% Systems with Gaussian Mixture Models", IEEE Transaction on Robotics, 2011.
%
% To get latest upadate of the software please visit
%                          http://lasa.epfl.ch/khansari
%
% Please send your feedbacks or questions to:
%                           mohammad.khansari_at_epfl.ch

%%
%checking if a fixed time step is provided or not.
if length(time)==1
    dt = time;
end

d = size(demos{1},1); %dimensionality of demosntrations
Data=[];

for i=1:length(demos)
    clear tmp half_cut_demos half_cut_time
    
    % de-noising data (not necessary)
        tmp(:) = smooth(imp{i}(:),25); 
        
        ind_first=find(tmp,1,'first');%find the 1 and 4 of point
        ind_last= find(tmp,1,'last');
        
        half_cut_demos=tmp(:,ind_first+1:ind_last-1);
        %cut data between 1 and 4 and find point 2 and 3
%         zero_index=find(half_cut_demos==0) %%挑出所有的0的位置
%         k=length(zero_index) %%所有0的个数
%         first_zero_index=zero_index(1); %%第一个0元素的位置
%         last_zero_index=zero_index(k); %%最后一个0元素的位置
        ind_second=find(half_cut_demos==0,1,'first');
        ind_third=find(half_cut_demos==0,1,'last');
        ind_second=ind_first+ind_second+1;
        ind_third=ind_first+ind_third+1;
        
    H=[ind_first ind_second-ind_first ind_third-ind_second ind_last-ind_third length(demos{i})-ind_last]
    
    demo_cut{i}=mat2cell(demos{i},[3],H)
    t_cut{i}=mat2cell(time{i},[1],H)
    imp_cut{i}=mat2cell(tmp,[1],H)
    
    data{i}=[t_cut{i};imp_cut{i};demo_cut{i}] %put into data{5}*{3*1};

    % saving demos next to each other
    Data = data;
end