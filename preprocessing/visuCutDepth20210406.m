clear all; close all; clc

%% loop through each file 
straight_cuts = {};
cut = 1;
files = dir('../data/preprocessed_transformed_data/april/depth/apple/straight/*_cut_*.json');
for file = files'    
    % find the name of the surface
    surfname = extractSurfaceName(file.name);
    if isempty(surfname)
        continue
    end
    
    % load the trial data
    in = fopen(fullfile(file.folder, file.name));
    message = jsondecode(fgetl(in));
    time = [];
    ee_pos = [];
    ee_vel = [];
    force = [];
    depth = [];
    while ~feof(in)
        message = jsondecode(fgetl(in));
        if strcmp(message.control.phase,'cut')
            if message.model.depth <= 0
                time = [time; message.time];
                depth = [depth; message.model.depth];
                for body=1:length(message.raw.bodies)
                    if strcmp(message.raw.bodies{body}.frame,'robot') && strcmp(message.raw.bodies{body}.name,'ee')
                        ee_pos = [ee_pos; message.raw.bodies{body}.pose.position'];
                        ee_vel = [ee_vel; message.raw.bodies{body}.twist.linear'];
                    end
                end
                for body=1:length(message.filtered.bodies)
                    if strcmp(message.filtered.bodies{body}.frame,'robot') && strcmp(message.filtered.bodies{body}.name,'ft_sensor')
                        force = [force; message.filtered.bodies{body}.wrench.force'];
                    end
                end
            end
        end
    end
    fclose(in);
    straight_cuts{cut} = [time depth ee_pos ee_vel force];
    cut = cut + 1;
end


%%
figure;
hold on;grid on;
for cut=1:length(straight_cuts)
    data = straight_cuts{cut};
    plot3(data(:,6), data(:,9), data(:,2));
end
xlabel('twist X')
ylabel('force X')
zlabel('depth')

%%
function surfname = extractSurfaceName(filename)
    surfname = regexp(filename, '20210406_(.*?_\d\d)_.*', 'tokens');
    if ~isempty(surfname)
        surfname = surfname{1}{1};
    end
end