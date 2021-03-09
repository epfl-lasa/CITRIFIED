clc; clear all; close all;

freq = 9.380037948039710e+02; % this is the sampling frequency of the raw data from february

[b, a] = butter(4, 6/freq*2); % create a Butterworth 1D filter with cutoff frequency 6Hz
                              % 6/freq*2 is the normalized cutoff frequency

input = [1, 3, 6, 5, 3, 7, 8, 9, 13, 10, 8, 7, 5, 7, 8, 10];

output = filter(b, a, input);