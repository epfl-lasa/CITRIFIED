%% this code deal with the zessis insertion in february
pwd;
currentFolder = pwd;
if isunix
    cd /home/rui/matlab
%     cd /home/wr/MATLAB_ON_LINUX
    addpath(genpath('WR_matlab_function'))
    addpath(genpath('8_Learn_DS_cut_task'))
    cd 8_Learn_DS_cut_task/7CITRIFIED/
else
    cd D:\matlab2017\work
    addpath(genpath('WR_matlab_function'))
    addpath(genpath('8_Learn_DS_cut_task'))
    cd 8_Learn_DS_cut_task\7CITRIFIED\
end

clc;clear all; close all;
%% choice data set

data_of_exp_input='february_new';
data_of_exp_output='february_new_learn';

phase_choice=[1:2];

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  0 define color and path
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
color_list=jet(3);

if isunix
    path_of_load = ['./data/processed_data/' data_of_exp_input '/'];
    path_of_save = ['./data/data_set4learn/' data_of_exp_output '/'];
    path_of_plot= ['./data/figure_for_paper/' data_of_exp_output '/'];
else
    path_of_load = ['.\data\processed_data\' data_of_exp_input '\'];
    path_of_save = ['.\data\data_set4learn\' data_of_exp_output '\'];
    path_of_plot= ['.\data\figure_for_paper\' data_of_exp_output '\'];
end

status = mkdir(path_of_save); 
status = mkdir(path_of_plot); 

%% load data

for exp_kind=1:3
    close all;
    % clear all;
    clearvars -except phase_choice max_phase1 max_phase2 max_phase3 cut_or_segment process_method exp_kind color_list data_of_exp path_of_load path_of_save path_of_plot;
    clc;
    if exp_kind==1
        exp_name=['apple']
        exp_time_good=[1:34];
        color=color_list(exp_kind,:);
    elseif exp_kind==2
        exp_name=['banana']
        exp_time_good=[1:11];
        color=color_list(exp_kind,:);
    elseif exp_kind==3
        exp_name=['orange']
        exp_time_good=[1:23];
        color=color_list(exp_kind,:);
    end
    
    %% load data file
    if isunix
        %%% for linux %%%%%%%%%%%%%%%%%%%%%%%%%
        dirs=dir([path_of_load exp_name '/*']');
    else
        dirs=dir([path_of_load exp_name '\*']');
    end
    dircell=struct2cell(dirs)';
    filenames=dircell(:,1);
    total_exp_time=length(filenames);
    data_name_set=[];
    for exp_time=1:total_exp_time-2
        name_of_exp=[filenames{exp_time+2,1}];
        C = strsplit(name_of_exp,'_');

        test_trail_name=[C{1} '_' C{2} '_' C{3} '_' C{4} '_' C{5} '_' C{6}];

        data_name_set{exp_time,1}=test_trail_name;
%         data_name_set{exp_time,2}=result_of_exp;
    end

    clear result_of_exp C name_of_exp name_of_exp dircell dirs filenames

    for exp_num=exp_time_good
        
        exp_num_name=char(data_name_set(exp_num));

        if isunix
            %%% for linux %%%%%%%%%%%%%%%%%%%%%%%%%
            filename=[path_of_load exp_name '/' exp_num_name];
        else
            filename=[path_of_load exp_name '\' exp_num_name];
        end
        
        delimiter = ',';
        startRow = 2;
        formatSpec = '%f%f%f%f%f%f%f%f%f%f%f%f%[^\n\r]';
        fileID = fopen(filename,'r');
        dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'EmptyValue' ,NaN,'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
        fclose(fileID);
        segmented_data = [dataArray{1:end-1}];
        clearvars filename delimiter startRow formatSpec fileID dataArray ans;
        
        fs=round((length(segmented_data(:,1))-1)/(segmented_data(end,1)-segmented_data(1,1)));
        time=segmented_data(:,1)';
        force=segmented_data(:,2:4)';
        pose=segmented_data(:,5:7)';
        real_vel=segmented_data(:,8:10)';
%         force_d=segmented_data(:,2:4)';
%         imp=[segmented_data(:,2:4)';segmented_data(:,2:4)'];
        phase_num=segmented_data(:,12)';
        
        %% fliter and get deritive 
        freq = 9.380037948039710e+02; % this is the sampling frequency of the raw data from february
        [b, a] = butter(4, 6/freq*2); % create a Butterworth 1D filter with cutoff frequency 6Hz
                                      % 6/freq*2 is the normalized cutoff frequency

        for j=1:3
            force_filted(j,:) = filter(b, a, force(j,:));
            real_vel_filted(j,:) = filter(b, a, real_vel(j,:));
        
            force_d(j,1)=0;force_d(j,2)=0;
            for i=3:length(phase_num)
                force_d(j,i)=((force_filted(j,i)-force_filted(j,i-2))*fs)/2;
                acc(j,i)=((real_vel_filted(j,i)-real_vel_filted(j,i-2))*fs)/2;
            end
        end
        
        
        
        %% segment data into different phase

        for phase=phase_choice
            if phase==1
                A=find(phase_num==0);
            elseif phase==2
                A=find(phase_num==1);
            elseif phase==2
                A=find(phase_num==2);
            elseif phase==2
                A=find(phase_num==3);
            end
            time_serise_data_serial{phase,exp_num}=time(:,A);
            pos_data_serial_norm_time{phase,exp_num}=[time(:,A);pose(:,A);real_vel_filted(:,A)];
            vel_knife_data_serial_norm_time{phase,exp_num}=[time(:,A);real_vel_filted(:,A);acc(:,A)];
            force_data_serial_norm_time{phase,exp_num}=[time(:,A);force_filted(:,A);force_d(:,A)];
            imp_data_serial_norm_time{phase,exp_num}=[time(:,A);force_filted(:,A);force_d(:,A)];
            getXx{phase,exp_num}= A(end);
        end
        clear fs time force pose real_vel phase_num  force_filted real_vel_filted force_d acc
        
        %% find out the insertion point and save
        
        
        if isunix
            status = mkdir([path_of_save 'cut_point/']); 
            filename1=[path_of_save 'cut_point/' exp_name num2str(exp_num) num2str(phase) 'choice_label_by_handgetXx.mat'];   
        else
            status = mkdir([path_of_save 'cut_point\']); 
            filename1=[path_of_save 'cut_point\' exp_name num2str(exp_num) num2str(phase) 'choice_label_by_handgetXx.mat'];   

        end
        save(filename1,'getXx');
        !echo save choice_label_by_hand done!


    end
    
    %% put data into my fasion

    if isunix
        status = mkdir([path_of_save 'all_3_phase/']); 
        filename1=[path_of_save 'all_3_phase/' exp_name '_all_data_three_phase.mat'];   
    else
        status = mkdir([path_of_save 'all_3_phase\']); 
        filename1=[path_of_save 'all_3_phase\' exp_name '_all_data_three_phase.mat'];   

    end
    save(filename1,'time_serise_data_serial','pos_data_serial_norm_time','vel_knife_data_serial_norm_time','force_data_serial_norm_time','imp_data_serial_norm_time')
            !echo save done
            
    clear time_serise_data_serial pos_data_serial_norm_time vel_knife_data_serial_norm_time force_data_serial_norm_time imp_data_serial_norm_time 

end







