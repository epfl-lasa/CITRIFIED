clear all; close all; clc;
load 'train_data.mat'
load 'test_data.mat'

X = train_data(:,1);
y = train_data(:,2);

samples = fopen('samples.dat', 'w');
fprintf(samples, '%f\n', X);
fclose(samples);

observations = fopen('observations.dat', 'w');
fprintf(observations, '%f\n', y);
fclose(observations);

kernel = fopen('kernel_params.dat', 'w');
fprintf(kernel, '%f\n', 0.7361);
fprintf(kernel, '%f\n', 0.1352);
fclose(kernel);
