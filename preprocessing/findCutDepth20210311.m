%% transforms

Transforms = struct();

files = dir('../data/raw_data/march/*_surface.csv');
for file = files'
    surface = importdata(fullfile(file.folder, file.name));
    
    % find the average transform of the robot base in the optitrack frame
    R = quat2rotm(mean(surface.data(:,5:8)));
    T = [R mean(surface.data(:,2:4))'; 0 0 0 1];
    
    surfname = extractSurfaceName(file.name);
    if isempty(surfname)
        continue
    end
    
    Transforms.(surfname) = T;
end

%% depth extraction
% surface meshgrid properties
res = 0.0015; % Grid Resolution
[xq,yq] = meshgrid(-0.6:res:0.6,-0.4:res:0.8);

drawPlot = true;

% loop through each file 
files = dir('../data/raw_data/march/*_cut_*.csv');
for file = files'
    
    % find the name of the surface
    surfname = extractSurfaceName(file.name);
    if isempty(surfname)
        continue
    end
    
    % load the associated surface data
    surfFile = dir(['../data/raw_data/march_depth/SurfaceReconstruction/*' surfname '*']);
    if numel(surfFile) ~= 1
        continue
    end
    surfaceData = load(fullfile(surfFile.folder, surfFile.name));
    
    trialname = regexp(file.name, 'cut_\d\d', 'match');
    
    fprintf('Processing %s, %s\n', surfname, trialname{1});
    
    % load the trial data
    trial = importdata(fullfile(file.folder, file.name));
    eePos = trial.data(:, 2:4);
    
    % transform the end-effector from robot frame to optitrack frame
    eePosInOptitrack = (Transforms.(surfname) * [eePos ones(size(eePos, 1), 1)]')';
    
    % subtract the average magic calibration number that Caroline determined
    % (distance to knife-tip)
    eePosInOptitrack = eePosInOptitrack(:, 1:3) - [0, 0, 0.1112];
    
    % extract the depth
    surfHeight = interp2(xq, yq, surfaceData.filtFit, eePosInOptitrack(:, 1), eePosInOptitrack(:, 2));
    depth = eePosInOptitrack(:, 3) - surfHeight;
    
    % store and write the csv back out
%     trial.data = [trial.data, depth];
%     trial.colheaders = [trial.colheaders, 'depth'];
%     headerRow = strjoin(trial.colheaders, ', ');
%     
%     outfile = fullfile('20210311_depth_trials_depth_reconstruction', file.name);
%     fid = fopen(outfile, 'wt');
%     if fid ~= -1
%         fprintf(fid, '%s\n', headerRow);
%     end
%     fclose(fid);
%     dlmwrite(outfile, trial.data, '-append');
    
    % plot
    if drawPlot
        figure(1); clf(1)
        subplot(2,1,1);
        m = mesh(xq, yq, surfaceData.filtFit);
        m.FaceColor = 'interp';
        m.FaceAlpha = 0.8;
        m.EdgeColor = [0 0 0];
        m.EdgeAlpha = 0.5;

        hold on;
        sc = scatter3(eePosInOptitrack(:,1), eePosInOptitrack(:,2), eePosInOptitrack(:,3));
        sc.CData = zeros(size(eePos, 1), 3);    % make RGB color zero (black)
        sc.CData(:,1) = abs(trial.data(:, 30)); % set R value to force magnitude
        sc.Marker = '.';
        hold off;

        axis equal;
        %     axis vis3d;
        view(45, 30);
        xlim([0.0 0.15]);
        ylim([-0.2 -0.05]);
        zlim([0.05 0.15]);

        title(strrep(strcat(surfname, '-', trialname{1}), '_', '-'));

        subplot(2,1,2);
        yyaxis('left');
        plot(trial.data(:,1), trial.data(:, 28), 'r', 'LineWidth', 2);
        hold on;
        plot(trial.data(:,1), trial.data(:, 29), 'g', 'LineWidth', 2);
        plot(trial.data(:,1), trial.data(:, 30), 'b', 'LineWidth', 2);
        hold off;
        ylim([-3, 3]);
        yyaxis('right');
        plot(trial.data(:,1), depth, 'k', 'LineWidth', 2);
        ylim([-0.01, 0.01]);
    end
end

function surfname = extractSurfaceName(filename)
    surfname = regexp(filename, '20210311_(.*?_\d\d)_.*', 'tokens');
    if ~isempty(surfname)
        surfname = surfname{1}{1};
    end
end