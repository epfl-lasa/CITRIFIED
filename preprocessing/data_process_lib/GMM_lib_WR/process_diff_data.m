function [Data] = process_diff_data(demos,time,imp)
%
% This function preprocess raw data and cut them in difference part as 
% format suitable for SEDS. The function cut the data of time and 
% trajectory based on the imp value, and the calculation is based on the 
% SEDS/ preprocess_demos. The function can be called using: 
%
%          [x0 , xT, Data, index] = cut2d_by_imp(demos,time,imp)
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
    clear tmp
    % de-noising data (not necessary)
%         tmp(1,:) = smooth(imp{i}(1,:),200); 
%         tmp(2,:) = smooth(imp{i}(2,:),200);
%         tmp(3,:) = smooth(imp{i}(3,:),200);      
    for j=1:d
        tmp(j,:) = smooth(demos{i}(j,:),25); 
    end
    
    demo_cut{i}=mat2cell(demos{i},[size(demos{i}(:,1),1)])
    t_cut{i}=mat2cell(time{i},[size(time{i}(:,1),1)])
    tmp_d = diff(tmp,1,2)/dt;
   
    
%     data{i}=[t_cut{i};imp_cut{i};demo_cut{i}] %put into data{5}*{3*1};
    data{i}=[demo_cut{i}{1};imp_cut{i}{1}]
    % saving demos next to each other
    Data = data;
end