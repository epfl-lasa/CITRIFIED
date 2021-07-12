clear all; close all; clc

files = dir('../data/raw_data/april/depth/*apple*cut*.json');
for file = files'
    in = fopen(fullfile(file.folder, file.name));
    message = jsondecode(fgetl(in));
    ee_pos = [];
    ee_pos_filt = [];
    time = [];
    while ~feof(in)
        message = jsondecode(fgetl(in));
        time = [time; message.time];
        for body=1:length(message.raw.bodies)
            if strcmp(message.raw.bodies{body}.frame,'robot') && strcmp(message.raw.bodies{body}.name,'ee')
                ee_pos = [ee_pos; message.raw.bodies{body}.pose.position'];
            end
        end
        for body=1:length(message.filtered.bodies)
            if strcmp(message.filtered.bodies{body}.frame,'robot') && strcmp(message.filtered.bodies{body}.name,'ee')
                ee_pos_filt = [ee_pos_filt; message.filtered.bodies{body}.pose.position'];
            end
        end
    end
    figure(1)
    plot(time,ee_pos(:,1))
    grid on; hold on;
    plot(time,ee_pos_filt(:,1))
    ylabel('Position X [m]')
    xlabel('Time [s]')
end
