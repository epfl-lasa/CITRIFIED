clear all; close all; clc;
%%
load 'train_data.mat'

test = importfile("/home/dominic/git/CITRIFIED/ml-tests/limbo/myGP/test.csv", [1, Inf]);
std = test(:,2);

figure
plot(train_data(:,1), train_data(:,2), '.')
hold on;
plot(train_data(:,1), test(:,1))
plot(train_data(:,1), test(:,1) + std, '--')
plot(train_data(:,1), test(:,1) - std, '--')