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

%% Time Windows

% Split data into time windows

length_tw = 3500; % number of samples in one time window
overlap = 0; % number of samples that must overlap
length_tw = length_tw + overlap;

vel_x_tw = buffer(vel_x, length_tw, overlap, 'nodelay');
vel_z_tw = buffer(vel_z, length_tw, overlap, 'nodelay');
force_x_tw = buffer(force_x, length_tw, overlap, 'nodelay');
force_z_tw = buffer(force_z, length_tw, overlap, 'nodelay');
time_tw = buffer(time, length_tw, overlap, 'nodelay');

% Store input data in cell arrays

nb_tw = length(time_tw(1,:));
timewindows_data = {};

for i = 1:nb_tw
    
    input.time = time_tw(:,i);
    input.force_x = force_x_tw(:,i);
    input.force_z = force_z_tw(:,i);
    input.vel_x = vel_x_tw(:,i);
    input.vel_z = vel_z_tw(:,i);
    
    % Derivative of Fx and Fz
    
    input.force_dt_x = zeros(length(input.force_x),1);
    input.force_dt_z = zeros(length(input.force_z),1);
    
    dt_fx = mean(diff(input.time));
    input.force_dt_x(2:end-1) = (input.force_x(3:end) - input.force_x(1:end-2)) / (2*dt_fx);
    input.force_dt_x(1) = input.force_x(2);
    input.force_dt_x(end) = input.force_x(end-1);
    
    dt_fz = mean(diff(input.time));
    input.force_dt_z(2:end-1) = (input.force_z(3:end) - input.force_z(1:end-2)) / (2*dt_fz);
    input.force_dt_z(1) = input.force_z(2);
    input.force_dt_z(end) = input.force_z(end-1);
    
    % Add input struct to cell array
    
    timewindows_data{1,i} = input;
    
end
