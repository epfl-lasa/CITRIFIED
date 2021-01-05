% Downsample - Amaury Wei - 03/06/2020

function [X_downsampled, y_downsampled] = downsample_data(X, y, nb_points)
%DOWNSAMPLE_DATA Downsample a dataset to train a model

NB_BINS = 15;    % Number of bins to decompose the x-values into
NB_POINTS_PER_BIN = round(nb_points / NB_BINS);

% Downsample the training set
min_displacement = 0;
delta = (max(X{:,1}) - min_displacement) / NB_BINS;
new_idx = [];
for bin = 1:NB_BINS
    lower = min_displacement + (bin-1)*delta;
    upper = min_displacement + bin*delta;
    idx = find(X{:,1} >= lower & X{:,1} < upper);
    if length(idx) > NB_POINTS_PER_BIN
        new_idx = [new_idx; sort(randi([1, length(X{:,1})], NB_POINTS_PER_BIN,1))];
    else
        new_idx = [new_idx; sort(idx)];
    end
end
X_downsampled = X(sort(new_idx),:);
y_downsampled = y(sort(new_idx),:);

end