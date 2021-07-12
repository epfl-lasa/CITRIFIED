clc; close all; clear all;

data_path = '../../data/raw_data/april/esn_finish_test/';
dir_content = struct2cell(dir([data_path '*cut*.json']'));
time = [];

for nb_file=1:size(dir_content, 2)
    nb_file
    file = fopen([data_path dir_content{1,nb_file}]);
    jsondecode(fgetl(file));
    while ~feof(file)
        message = jsondecode(fgetl(file));
%         if strcmp(message.control.phase,'cut') && ~triggered
%             triggered = message.time;
%         end
        if strcmp(message.control.phase,'cut')
            time = [time; message.time - triggered];
        end
    end
end

mean(diff(time))