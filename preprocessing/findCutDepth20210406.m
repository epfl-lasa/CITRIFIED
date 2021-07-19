clear all; close all; clc
%% surface

% files = dir('../data/raw_data/april_new_surface/*_surface.json');
% for file = files'
%     in = fopen(fullfile(file.folder, file.name));
%     message = jsondecode(fgetl(in));
%     touch_points = [];
%     while ~feof(in)
%         message = jsondecode(fgetl(in));
%         touch_points = [touch_points; message.control.touch_position'];
%     end
%     top = touch_points(1,1:2);
%     touch_points = touch_points(2:end,:);
%     res = 0.002;
%     [xq,yq] = meshgrid(top(1)-0.03:res:top(1)+0.03, top(2)-0.03:res:top(2)+0.03);
%     
%     fitSurface = griddata(touch_points(:,1), touch_points(:,2), touch_points(:,3), xq, yq, 'natural');
%     
%     figure(1);
%     p = plot3(touch_points(:,1), touch_points(:,2), touch_points(:,3), 'k:');
%     p.LineWidth = 0.5;
%     hold on;
%     m1 = mesh(xq, yq, fitSurface);
%     m1.FaceColor = 'interp';
%     m1.FaceAlpha = 0.2;
%     m1.EdgeColor = [0 0 0];
%     title('Raw surface data and mesh');
% end

%% depth extraction
drawPlot = false;

% loop through each file 
files = dir('../data/raw_data/april/depth/*_cut_*.json');
for file = files'
    
    trialname = regexp(file.name, 'cut_\d\d', 'match');
    
    % find the name of the surface
    surfname = extractSurfaceName(file.name);
    if isempty(surfname)
        continue
    end
    if strcmp(surfname, 'orange_03') && strcmp(trialname{1},'cut_05')
        continue
    end
    if strcmp(surfname, 'apple_01') && strcmp(trialname{1},'cut_03')
        continue
    end
    
    % load the associated surface data and fit surface
    surfFile = dir(['../data/raw_data/april/depth/20210406_' surfname '_surface*']);
    if numel(surfFile) ~= 1
        continue
    end
    surf_in = fopen(fullfile(surfFile.folder, surfFile.name));
    message = jsondecode(fgetl(surf_in));
    touch_points = [];
    while ~feof(surf_in)
        message = jsondecode(fgetl(surf_in));
        touch_points = [touch_points; message.control.touch_position'];
    end
    fclose(surf_in);
    top = touch_points(1,1:2);
    touch_points = touch_points(2:end,:);
    res = 0.002;
    [xq,yq] = meshgrid(top(1)-0.03:res:top(1)+0.03, top(2)-0.03:res:top(2)+0.03);
    fitSurface = griddata(touch_points(:,1), touch_points(:,2), touch_points(:,3), xq, yq, 'natural');
    
    fprintf('Processing %s, %s\n', surfname, trialname{1});
    
    % load the trial data
    in = fopen(fullfile(file.folder, file.name));
    out = fopen(fullfile(file.folder, 'depth', file.name), 'w');
    message = jsondecode(fgetl(in));
    fwrite(out,[jsonencode(message) sprintf('\r')]);
    time = [];
    ee_pos = [];
    force = [];
    while ~feof(in)
        message = jsondecode(fgetl(in));
        if strcmp(message.control.phase,'cut')
            time = [time; message.time];
            for body=1:length(message.raw.bodies)
                if strcmp(message.raw.bodies{body}.frame,'robot') && strcmp(message.raw.bodies{body}.name,'ee')
                    ee_pos = [ee_pos; message.raw.bodies{body}.pose.position'];
                end
            end
            for body=1:length(message.filtered.bodies)
                if strcmp(message.filtered.bodies{body}.frame,'robot') && strcmp(message.filtered.bodies{body}.name,'ft_sensor')
                    force = [force; message.filtered.bodies{body}.wrench.force'];
                end
            end
            message.model.depth = ee_pos(end,3) - interp2(xq, yq, fitSurface, ee_pos(end, 1), ee_pos(end, 2));
        end
        fwrite(out,[jsonencode(message) sprintf('\r')]);
    end
    fclose(in);
    fclose(out);
    
    % extract the depth
    surfHeight = interp2(xq, yq, fitSurface, ee_pos(:, 1), ee_pos(:, 2));
    depth = ee_pos(:, 3) - surfHeight;
        
    % plot
    if drawPlot
        figure(1); clf(1)
        subplot(2,1,1);
        m = mesh(xq, yq, fitSurface);
        m.FaceColor = 'interp';
        m.FaceAlpha = 0.8;
        m.EdgeColor = [0 0 0];
        m.EdgeAlpha = 0.5;

        hold on;
        sc = scatter3(ee_pos(:,1), ee_pos(:,2), ee_pos(:,3));
        sc.CData = zeros(size(ee_pos, 1), 3);    % make RGB color zero (black)
        sc.CData(:,1) = abs(force(:,1)); % set R value to force magnitude
        sc.Marker = '.';
        hold off;

        axis equal;
%             axis vis3d;
        view(45, 30);
        xlim([top(1)-0.03 top(1)+0.03]);
        ylim([top(2)-0.03 top(2)+0.03]);
%         zlim([0.05 0.15]);

        title(strrep(strcat(surfname, '-', trialname{1}), '_', '-'));

        subplot(2,1,2);
        yyaxis('left');
        plot(time, force(:,1), 'r', 'LineWidth', 2);
        hold on;
        plot(time, force(:,2), 'g', 'LineWidth', 2);
        plot(time, force(:,3), 'b', 'LineWidth', 2);
        hold off;
        ylim([-5, 5]);
        yyaxis('right');
        plot(time, depth, 'k', 'LineWidth', 2);
        ylim([-0.01, 0.01]);
    end
end

function surfname = extractSurfaceName(filename)
    surfname = regexp(filename, '20210406_(.*?_\d\d)_.*', 'tokens');
    if ~isempty(surfname)
        surfname = surfname{1}{1};
    end
end