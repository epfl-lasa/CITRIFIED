clc
clear all
close all

% Function inputs 

filename = '20210330_orange_03_insertion_04.json';
TYPE = 'force'; % velocity, force, position
SIGNAL = 'filtered'; % raw, filtered

% Function outputs - phase, esn, force_raw, force_filt, ...
% ... vel_raw, vel_filt, pos_raw, t

[t] = data_visualization(filename,TYPE,SIGNAL);