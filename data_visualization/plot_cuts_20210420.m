close all; clear all; clc

data_path = '../data/raw_data/april/depth_controlled_test/';
dir_content = struct2cell(dir([data_path '*cut*.json']'));

for nb_file=1:size(dir_content, 2)
    nb_file
    pos = [];
    ee_pos = [];
    ee_pos_time = [];
    t = [];
    touch = [];
    depth = [];
    depth_offset = [];
    depth_time = [];
    file = fopen([data_path dir_content{1,nb_file}]);
    message = jsondecode(fgetl(file));
    while ~feof(file)
        message = jsondecode(fgetl(file));
        t = [t; message.time];
        if isfield(message, 'raw')
            for b = message.raw.bodies'
                if strcmp(b{1}.frame, 'task') && strcmp(b{1}.name, 'ee')
                    pos = [pos; b{1}.pose.position'];
                end
                if strcmp(b{1}.frame, 'robot') && strcmp(b{1}.name, 'task')
                    depth_offset = b{1}.pose.position(3);
                end
                if strcmp(b{1}.frame, 'task') && strcmp(b{1}.name, 'ee')
                    ee_pos = [ee_pos; b{1}.pose.position(3)];
                    ee_pos_time = [ee_pos_time; message.time];
                end
            end
        end
        if isfield(message, 'model') && isfield(message.model, 'touch_position')
            touch = [touch; message.model.touch_position'];
        end
        if isfield(message, 'model') && isfield(message.model, 'depth')
            if ~strcmp(message.control.phase,'insertion') && ~strcmp(message.control.phase,'pause')
                depth = [depth; message.model.depth + depth_offset];
            else
                depth = [depth; message.model.depth];
            end
            depth_time = [depth_time; message.time];
        end
    end
%     figure
%     plot3(pos(:,1),pos(:,2),pos(:,3))
%     hold on;
%     scatter3(touch(:,1), touch(:,2), touch(:,3), 'x');
%     hold off;
%     axis equal
%     axis vis3d
%     title('Position','interpreter','latex')
%     xlabel('x','interpreter','latex')
%     ylabel('y','interpreter','latex')
%     zlabel('z','interpreter','latex')

%     depth_time(depth>0.1) = [];
%     depth(depth>0.1) = [];
    figure
    plot(depth_time, 1e3*depth)
    hold on; grid on;
    plot([depth_time(1) depth_time(end)], 1e3*[0.005 0.005])
    xlabel('time','interpreter','latex')
    ylabel('depth','interpreter','latex')
    
    figure
    plot(ee_pos_time, ee_pos)
end

