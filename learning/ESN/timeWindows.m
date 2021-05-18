clc; clear all;

%% Parse the JSON data

filename = 'silicon_04_insertion_01.json';

file = fopen(filename);

phase = {};

force_x = [];
force_z = [];
vel_x = [];
vel_z = [];
time = [];
depth = [];


while ~feof(file)
    
    line = fgetl(file);
    message = jsondecode(line);
    
    if isfield(message, 'control')
        
        if strcmp(message.control.phase,'insertion')
            
            phase{end+1} = message.control.phase;
            
            if isfield(message, 'filtered')
                vel_x(end+1) = message.filtered.bodies{1,1}.twist.linear(1)';
                vel_z(end+1) = message.filtered.bodies{1,1}.twist.linear(3)';
            end
            
            if isfield(message, 'filtered')
                force_x(end+1,1) = message.filtered.bodies{3,1}.wrench.force(1);
                force_z(end+1,1) = message.filtered.bodies{3,1}.wrench.force(3);
            end
            
            if isfield(message, 'time')
                time = [time, message.time];
            end
            
            if isfield(message, 'depth')
                depth = [depth, message.model.depth];
            end
            
        end
        
    end
    
    
end


fclose(file);

force_dt_x = zeros(length(force_x),1);
force_dt_z = zeros(length(force_z),1);

dt_fx = mean(diff(force_x));
force_dt_x(2:end-1) = (force_x(3:end) - force_x(1:end-2)) / dt_fx;
force_dt_x(1) = force_x(2);
force_dt_x(end) = force_x(end-1);

dt_fz = mean(diff(force_z));
force_dt_z(2:end-1) = (force_z(3:end) - force_z(1:end-2)) / dt_fz;
force_dt_z(1) = force_z(2);
force_dt_z(end) = force_z(end-1);

%% Time Windows

% Split data into time windows

nb_tw = 3; % number of time windows
overlap = 0; % number of samples that must overlap
length_tw = ceil((length(vel_x) + (overlap * nb_tw)) / nb_tw); % lenth of each time window

vel_x_tw = buffer(vel_x, length_tw, overlap, 'nodelay');
vel_z_tw = buffer(vel_z, length_tw, overlap, 'nodelay');
force_x_tw = buffer(force_x, length_tw, overlap, 'nodelay');
force_z_tw = buffer(force_z, length_tw, overlap, 'nodelay');
force_dt_x_tw = buffer(force_dt_x, length_tw, overlap, 'nodelay');
force_dt_z_tw = buffer(force_dt_z, length_tw, overlap, 'nodelay');
time = buffer(time, length_tw, overlap, 'nodelay');

% Store input data in cell arrays

timewindows_data = {};

for i = 1:nb_tw
    
    input.time = time(:,i);
    input.force_x = force_x_tw(:,i);
    input.force_z = force_z_tw(:,i);
    input.vel_x = vel_x_tw(:,i);
    input.vel_z = vel_z_tw(:,i);
    input.force_derivative_x = force_x_tw(:,i);
    input.force_derivative_z = force_x_tw(:,i);
    
    timewindows_data{1,i} = input;
    
end
