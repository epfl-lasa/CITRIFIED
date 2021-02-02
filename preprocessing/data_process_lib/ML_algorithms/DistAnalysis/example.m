% example on how to use mDistAn

%% generate data

% define the number of samples for each calss
nb_samples = 20;

% set the mean and the sigma of class 1
mu1 = [1 1 1 5];
sigma = [0.5 0 0 0; 0 0.5 0 0; 0 0 0.5 0; 0 0 0 0.5];

% generate random data for class 1 based on the defined distribution
data1 = generateData(nb_samples, 'dims', length(mu1), 'mu', mu1, 'sigma', sigma);
labels1 = ones(nb_samples, 1);

% repeat the steps for classes 2 and 3
mu2 = [-2 10 -2 5];
sigma2 = [0.7 0 0 0; 0 0.7 0 0; 0 0 0.5 0; 0 0 0 0.5];
data2 = generateData(nb_samples, 'dims', length(mu2), 'mu', mu2, 'sigma', sigma2);
labels2 = ones(nb_samples, 1) * 2;

mu3 = [3 3 3 12];
data3 = generateData(nb_samples, 'dims', length(mu3), 'mu', mu3, 'sigma', sigma);
labels3 = ones(nb_samples, 1) * 3;

% concatenate all the data and labels to generate the overall dataset
mdata = [data1; data2; data3];
mlabels = [labels1; labels2; labels3];

% plot the data for inspection
figure
scatter(mdata(mlabels == 1,1), mdata(mlabels == 1,2))
hold on
scatter(mdata(mlabels == 2,1), mdata(mlabels == 2,2))
scatter(mdata(mlabels == 3,1), mdata(mlabels == 3,2))
title("original data")
legend('class 1', 'class 3', 'class 3')

%% compute the model

mModel = mDistAn(mdata, mlabels);

%% make a prediction on the data

[predictedLabel, likelihood, confidence] = mPredict(mdata, mModel);


