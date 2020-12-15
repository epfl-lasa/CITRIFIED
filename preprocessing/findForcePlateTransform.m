function [T, Data] = findForcePlateTransform(path, showFigure)
% Find the FT-to-world transformation matrix
%
%   For a set of calibration data, in which there is an optitrack
%   and FT sensor data set containing some number of vertical
%   pressure measurments, calculate the transformation matrix to
%   convert force data from the sensor frame to the optitrack world frame.
%
%   Example calibration data can be found here:
%   https://drive.google.com/drive/folders/1sX-ajnjc_xIMf_L_6g38yl-gBkSbCy5o?usp=sharing
arguments
    path string = '../data/raw_data/calibration/force_plate_frame'
    showFigure logical = true
end

%% import data
ft = importdata(fullfile(path, 'ft_sensor.csv'));
optitrack = importdata(fullfile(path, 'optitrack.csv'));

% extract force data and correct for sensor bias
AVG_SAMPLES = 50;
force = ft.data(:, 4:6) - mean(ft.data(1:AVG_SAMPLES, 4:6));
torque = ft.data(:, 7:9) - mean(ft.data(1:AVG_SAMPLES, 7:9));
ftTime = ft.data(:, 3);

% get optitrack data
knifeTip = optitrack.data(:, 4:6);
knifeQ = optitrack.data(:, [10, 7:9]);
base = optitrack.data(:, 11:13);
baseQ = optitrack.data(:, [17, 14:16]);
optiTime = optitrack.data(:, 3);

% world frame to base frame 
baseTM = trvec2tform(mean(base)) * quat2tform(mean(baseQ));

% resample to common time base
timevec = max(ftTime(1), optiTime(1)):0.01:min(ftTime(end), optiTime(end));

knifeTipTS = timeseries(knifeTip, optiTime).resample(timevec);
knifeQTS = timeseries(knifeQ, optiTime).resample(timevec);

forceTS = timeseries(force, ftTime).resample(timevec);
torqueTS = timeseries(torque, ftTime).resample(timevec);

% store resampled calibration data
Data = struct('force', forceTS.Data, 'torque', torqueTS.Data, ...
    'position', knifeTipTS.Data, 'orientation', knifeQTS.Data, ...
    'time', timevec);

%% find high-force areas for center of pressure calculation

% range of time samples (in resampled time base) to include in the 
%   center of pressure calculations
calibrationRange = 200:length(Data.force)/2;

MAGIC_NUMBER = 2;

[cop, pos] = deal(zeros(0, 3)); %#ok<*AGROW>
for t = calibrationRange
    % above a force threshold, calculate center of pressure and store with
    % the relevant knife tip position
    if all(abs(Data.force(t, :)) > 0.2) && abs(Data.force(t, 3)) > 3
        cop(end+1, :) = ...
            [-Data.torque(t, 2) / Data.force(t, 3), ...
              Data.torque(t, 1) / Data.force(t, 3), Data.position(t, 3)];
        pos(end+1, :) = Data.position(t, 1:3);
    end
end
cop = cop * MAGIC_NUMBER;

% find rigid transform to fit cop data points onto knife positions
T = findTransform(cop', pos');


%% plot results

if showFigure
    copTransformed = (T * [cop'; ones(1, length(cop))])';

    figure()
    subplot(3,1,1)
    scatter(cop(:, 1), cop(:, 2), 'r');
    title('Center of pressure in force plate frame')
    legend('CoP');
    axis equal;

    subplot(3,1,2)
    scatter(pos(:, 1), pos(:, 2), 'b');
    title('Knife tip locations in optitrack world frame');
    legend('KnifeTip');
    axis equal;

    subplot(3,1,3)
    s1 = scatter(pos(:, 1), pos(:, 2), 'b');
    hold on;
    s2 = scatter(copTransformed(:, 1), copTransformed(:, 2), 'r');
    hold off;
    title('Center of pressure transformed to optitrack world frame')
    legend([s1, s2], {'KnifeTip', 'Transformed CoP'});
    axis equal;
end

end

% Find rigid transformation to best fit 3D indexed point sets using SVD
function [T, R, t] = findTransform(A, B)
    assert(all(size(A) == size(B)));
    
    Au = mean(A, 2);
    Bu = mean(B, 2);

    % subtract mean
    Am = A - repmat(Au, 1, size(A, 2));
    Bm = B - repmat(Bu, 1, size(B, 2));

    % calculate covariance matrix (is this the correct terminology?)
    H = Am * Bm';

    % find rotation
    [U,~,V] = svd(H);
    R = V*U';

    % correct reflection 
    if det(R) < 0
        V(:,3) = -V(:,3);
        R = V*U';
    end

    % find offset
    t = -R*Au + Bu;
    
    % construct transformation matrix
    T = [R, t; ...
         0, 0, 0, 1];
end
