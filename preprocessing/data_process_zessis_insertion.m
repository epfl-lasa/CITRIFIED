%% this code deal with the zessis insertion in february
pwd;
currentFolder = pwd;
if isunix
    cd /home/rui/matlab
    addpath(genpath('CITRIFIED'))
    cd CITRIFIED/
else
    cd D:\matlab2017\work
    addpath(genpath('CITRIFIED'))
    cd CITRIFIED\
end

clc;clear all; close all;
%% choice data set

data_of_exp_input='february';
data_of_exp_output='february_learn';

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
    clearvars -except max_phase1 max_phase2 max_phase3 cut_or_segment process_method exp_kind color_list data_of_exp path_of_load path_of_save path_of_plot;
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
        exp_time_good=[1:25];
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
        
        phase=1;
        
        if isunix
            %%% for linux %%%%%%%%%%%%%%%%%%%%%%%%%
            filename=[path_of_load exp_name '/' exp_num_name];
        else
            filename=[path_of_load exp_name '\' exp_num_name];
        end
        
        delimiter = ',';
        startRow = 2;
        formatSpec = '%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%[^\n\r]';
        fileID = fopen(filename,'r');
        dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'EmptyValue' ,NaN,'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
        fclose(fileID);
        segmented_data = [dataArray{1:end-1}];
        clearvars filename delimiter startRow formatSpec fileID dataArray ans;
        
        
        time=segmented_data(:,1)';
        force=segmented_data(:,2:4)';
        real_vel=segmented_data(:,5:7)';
        des_vel=segmented_data(:,8:10)';
        force_d=segmented_data(:,11:13)';
        phase_num=segmented_data(:,15)';
        
        A=find(phase_num<2);
        B=find(phase_num>0);
        
        time_serise_data_serial{phase,exp_num}=time(:,A);
        pos_data_serial_norm_time{phase,exp_num}=[time(:,A);real_vel(:,A)];
        vel_knife_data_serial_norm_time{phase,exp_num}=[time(:,A);real_vel(:,A)];
        force_data_serial_norm_time{phase,exp_num}=[time(:,A);force(:,A);force_d(:,A)];
        imp_data_serial_norm_time{phase,exp_num}=[time(:,A);force(:,A);force_d(:,A)];
        
        %% find out the insertion point and save
        
        getXx{phase,exp_num}= B(1);
        save([path_of_save 'all_train_model/' exp_name num2str(exp_num) num2str(phase) 'choice_label_by_handgetXx.mat'],'getXx');
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







