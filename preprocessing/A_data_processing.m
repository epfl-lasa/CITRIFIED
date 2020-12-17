%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Rui Wu 2020.12.15
%   process all data and get impedance estimate
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i=2:6
close all;
% clear all;
clearvars -except i;
clc;

% avocado time wrong
%banana good 6 force short shallow 4 data wrong
%
%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  1 choice dataset and set path
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
data_of_exp='curve_cut_1211';
task='avocado';
% task='banana';
% task='orange';

%%%%% auto
exp_name=['deep'];
% exp_name=['good'];
% exp_name=['shallow'];

% exp_time=['1'];
% exp_time=[num2str(str2num(exp_time)+1)]
exp_time=[num2str(i)]

plot_all_figure=1;


%%% for win %%%%%%%%%%%%%%%%%%%%%%%%%

filename_emg = ['.\data\raw_data\' data_of_exp '\' task '\' exp_name '\' exp_time '\emg.csv'];
filename_opt = ['.\data\raw_data\' data_of_exp '\' task '\' exp_name '\' exp_time '\optitrack.csv'];
filename_force = ['.\data\raw_data\' data_of_exp '\' task '\' exp_name '\' exp_time '\ft_sensor.csv'];

% path_of_save = ['.\preprocessing\data_after_cut\' data_of_exp '\'];
% path_of_plot= ['.\preprocessing\figure_for_paper\' data_of_exp '\'];
% %%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% status = mkdir(path_of_save); 
% status = mkdir(path_of_plot); 

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 2 load all data
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Import data from text file.
delimiter = ',';
startRow = 2;
formatSpec = '%f%f%f%f%f%f%f%f%f%f%f%f%[^\n\r]';
fileID = fopen(filename_emg,'r');
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'TextType', 'string', 'EmptyValue', NaN, 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
fclose(fileID);
approximatetime_data_EMG = [dataArray{1:end-1}];
clearvars filename_emg delimiter startRow formatSpec fileID dataArray ans;
%% opt
%%%%%%%%%%%%% without tip of knife %%%%%%%%%%%%%%%%%%%%
delimiter = ',';
startRow = 2;
formatSpec = '%f%f%f%f%f%f%f%f%f%f%*s%*s%*s%*s%*s%*s%*s%[^\n\r]';
fileID = fopen(filename_opt,'r');
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'TextType', 'string', 'EmptyValue', NaN, 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
fclose(fileID);
opt = [dataArray{1:end-1}];
clearvars filename_opt delimiter startRow formatSpec fileID dataArray ans;

%% force data
delimiter = ',';
startRow = 2;
formatSpec = '%f%f%f%f%f%f%f%f%f%[^\n\r]';
fileID = fopen(filename_force,'r');
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'TextType', 'string', 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
fclose(fileID);
ftsensor = [dataArray{1:end-1}];
clearvars filename_force delimiter startRow formatSpec fileID dataArray ans;


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 3 process all data
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 3.1 put each data into their own set
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% EMG
[ P16, raw_EMG_flitered] = EMG_process_9ch( approximatetime_data_EMG(:,3:end) );
EMG=P16;
pause
close all;

%% opt data
Position_hand=zeros(length(opt),4);
for i=1:length(opt)  
    Position_hand(i,1)=opt(i,3);
    Position_hand(i,2)=opt(i,4);
    Position_hand(i,3)=opt(i,5);
    Position_hand(i,4)=opt(i,6);
    Position_hand(i,5)=opt(i,7);
    Position_hand(i,6)=opt(i,8);
    Position_hand(i,7)=opt(i,9);
    Position_hand(i,8)=opt(i,10);
end 




Position_elbow=zeros(length(opt),4);
for i=1:length(opt)  
    Position_elbow(i,1)=opt(i,3);
    Position_elbow(i,2)=opt(i,7);
    Position_elbow(i,3)=opt(i,8);
    Position_elbow(i,4)=opt(i,9);
end 

%% Force
Force=zeros(length(ftsensor),4);
for i=1:length(ftsensor)  
    Force(i,1)=ftsensor(i,3);%+0.01;%+0.01
    Force(i,2)=ftsensor(i,4);%/1000; 
    Force(i,3)=ftsensor(i,5);%/1000;
    Force(i,4)=ftsensor(i,6);%/1000;
end

Force(:,2:4)=Force(:,2:4)-Force(1,2:4); 


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 3.2 deal with time begin and end together, cut useless data
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% cut the useless data at begin and end
scale_position=90;vel_scale=1;
f=figure;plot_wr(f,1);
plot(Position_elbow(:,1),(Position_elbow(:,4)-Position_elbow(1,4))*scale_position,'b-', ...
    Position_hand(:,1),(Position_hand(:,4)-Position_hand(1,4))*scale_position,'y-', ...
    Force(:,1),Force(:,4)-Force(1,4),'r-', ...
    EMG(:,1),(EMG(:,5)-EMG(1,5))*vel_scale,'g-', ...
    'LineWidth',1,'markersize',6);
title('comparision of postion and Force in X');xlabel('t/s');ylabel('x/m and F/mN'); 
legend('position-elbow-X','position-hand-X','Force-robot-X','EMG-X'); 
grid on;

[getxX,getxY]=ChoicePointByMouse();
time_begin=getxX(1);time_end=getxX(2);

Force_robot_cut=Force;
Position_elbow_cut=Position_elbow;Position_robot_cut=Position_hand;
EMG_cut=EMG;
idfr= Force_robot_cut(:,1)<time_begin | Force_robot_cut(:,1)> time_end;
Force_robot_cut(idfr,:)=[];
idpe= Position_elbow_cut(:,1)<time_begin | Position_elbow_cut(:,1)> time_end;
Position_elbow_cut(idpe,:)=[];
idpr= Position_robot_cut(:,1)<time_begin | Position_robot_cut(:,1)> time_end;
Position_robot_cut(idpr,:)=[];
ide= EMG_cut(:,1)<time_begin | EMG_cut(:,1)> time_end;
EMG_cut(ide,:)=[];

Force=Force_robot_cut;
Position_elbow=Position_elbow_cut;Position_hand=Position_robot_cut;
EMG=EMG_cut;


%% find the early data and get the Absolute time
if  Force(1,1)==min([Force(1,1),Position_hand(1,1),Position_elbow(1,1),EMG(1,1)])
        init_time=Force(1,1);
elseif Position_hand(1,1)==min([Force(1,1),Position_hand(1,1),Position_elbow(1,1),EMG(1,1)])
        init_time=Position_hand(1,1);
elseif Position_elbow(1,1)==min([Force(1,1),Position_hand(1,1),Position_elbow(1,1),EMG(1,1)])
        init_time=Position_elbow(1,1);
elseif EMG(1,1)==min([Force(1,1),Position_hand(1,1),Position_elbow(1,1),EMG(1,1)])
        init_time=EMG(1,1);
end

Force(:,1)=Force(:,1)-init_time;
Position_hand(:,1)=Position_hand(:,1)-init_time;
Position_elbow(:,1)=Position_elbow(:,1)-init_time;
EMG(:,1)=EMG(:,1)-init_time;



%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 3.3 filter position and force
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% position filter 
FilterOrNot=1;% filter before process  1 yes 0 no
filter_kind=0;%filter kind 0 low pass,1 band pass
low_cut_off=0.5;
high_cut_off=18;
plot_filter=0;
Position_tool_filter=Position_filter(Position_hand,FilterOrNot,filter_kind,low_cut_off,high_cut_off,plot_filter);
suptitle('Position_hand-raw band filter for imp iden');

Position_tool_filter=Position_filter(Position_elbow,FilterOrNot,filter_kind,low_cut_off,high_cut_off,plot_filter);
suptitle('Position_elbow-raw band filter for imp iden');


%% filter force
FilterOrNot=1;% filter before process  1 yes 0 no
filter_kind=0;%filter kind 0 low pass,1 band pass
low_cut_off=0.5;
high_cut_off=20;
plot_filter=0;
Force_robot_filter=Force_filter(Force,FilterOrNot,filter_kind,low_cut_off,high_cut_off,plot_filter);
suptitle('Force-robot band pass');

%% %%%%%%%%%%%%%%%%%%%%%%%%%5
%% 3.4 get MCV  of EMG
%% %%%%%%%%%%%%%%%%%%%%%
for i=2:length(EMG(1,:))
    RMS(i)=rms(EMG(:,i));
    MAX(i)=max(EMG(:,i));
end
for i=2:length(EMG(1,:))
    EMG(:,i)=500*EMG(:,i)/MAX(i);
end


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 3.5 chabu
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
EMG=downsample(EMG,8);

    Q=EMG(:,1);%:0.00005:EMG(end,1);

    %then interp the data with the time stamp of Q
    clear chabu_Hand chabu_Hand chabu_EMG chabu_Blade chabu_Force co_contravc_EMG_plot IMP
    chabu_Elbow(:,1)=Q(:,1);
    chabu_Elbow(:,2)=interp1(Position_elbow(:,1),Position_elbow(:,2),Q(:,1),'linear');%'nearest' 'spline' 'cubic'
    chabu_Elbow(:,3)=interp1(Position_elbow(:,1),Position_elbow(:,3),Q(:,1),'linear');%'nearest' 'spline' 'cubic'
    chabu_Elbow(:,4)=interp1(Position_elbow(:,1),Position_elbow(:,4),Q(:,1),'linear');%'nearest' 'spline' 'cubic'


    chabu_Hand(:,1)=Q(:,1);
    chabu_Hand(:,2)=interp1(Position_hand(:,1),Position_hand(:,2),Q(:,1),'linear');%'nearest' 'spline' 'cubic'
    chabu_Hand(:,3)=interp1(Position_hand(:,1),Position_hand(:,3),Q(:,1),'linear');%'nearest' 'spline' 'cubic'
    chabu_Hand(:,4)=interp1(Position_hand(:,1),Position_hand(:,4),Q(:,1),'linear');%'nearest' 'spline' 'cubic'
    chabu_Hand(:,5)=interp1(Position_hand(:,1),Position_hand(:,5),Q(:,1),'linear');%'nearest' 'spline' 'cubic'
    chabu_Hand(:,6)=interp1(Position_hand(:,1),Position_hand(:,6),Q(:,1),'linear');%'nearest' 'spline' 'cubic'
    chabu_Hand(:,7)=interp1(Position_hand(:,1),Position_hand(:,7),Q(:,1),'linear');%'nearest' 'spline' 'cubic'
    chabu_Hand(:,8)=interp1(Position_hand(:,1),Position_hand(:,8),Q(:,1),'linear');%'nearest' 'spline' 'cubic'
    
    chabu_Force(:,1)=Q(:,1);
    chabu_Force(:,2)=interp1(Force(:,1),Force(:,2),Q(:,1),'linear');%'nearest' 'spline' 'cubic'
    chabu_Force(:,3)=interp1(Force(:,1),Force(:,3),Q(:,1),'linear');%'nearest' 'spline' 'cubic'
    chabu_Force(:,4)=interp1(Force(:,1),Force(:,4),Q(:,1),'linear');%'nearest' 'spline' 'cubic'

chabu_EMG=EMG;



%%  %%%%%%%%%%%%%%%
%% 3.5.1 deal nan value
%% %%%%%%%%%%%%%%%%
ALL_data=[chabu_Hand chabu_Elbow chabu_Force chabu_EMG ];

ALL_data = ALL_data(all(ALL_data==ALL_data,2),:);  % a = a(~any(isnan(a),2),:); 
ALL_data(any(ALL_data~=ALL_data,2),:) = []; % a(any(isnan(a),2),:) = [];

chabu_Hand=ALL_data(:,1:8);
chabu_Elbow=ALL_data(:,9:12);
chabu_Force=ALL_data(:,13:16);
chabu_EMG=ALL_data(:,17:end);

%% compare the Force and position after interp
if plot_all_figure==1
%compare the data before and after interp
figure
subplot(311)
plot(chabu_Elbow(:,1),chabu_Elbow(:,2)-chabu_Elbow(1,2),'b-');hold on;
plot(Position_elbow(:,1),Position_elbow(:,2)-Position_elbow(1,2),'g-');hold on;
legend('chabu_Elbow','Position_elbow'); xlabel('t/s');ylabel('x/m and F/mN'); 
subplot(312)
plot(chabu_Elbow(:,1),chabu_Elbow(:,3)-chabu_Elbow(1,3),'b-');hold on;
plot(Position_elbow(:,1),Position_elbow(:,3)-Position_elbow(1,3),'g-');hold on;
legend('chabu_Elbow','Position_elbow'); xlabel('t/s');ylabel('x/m and F/mN'); 
subplot(313)
plot(chabu_Elbow(:,1),chabu_Elbow(:,4)-chabu_Elbow(1,4),'b-');hold on;
plot(Position_elbow(:,1),Position_elbow(:,4)-Position_elbow(1,4),'g-');hold on;
legend('chabu_Elbow','Position_elbow'); xlabel('t/s');ylabel('x/m and F/mN'); 
suptitle('chabu_Elbow');

%compare the data before and after interp
figure
subplot(311)
plot(chabu_Hand(:,1),chabu_Hand(:,2)-chabu_Hand(1,2),'b-');hold on;
plot(Position_hand(:,1),Position_hand(:,2)-Position_hand(1,2),'g-');hold on;
legend('chabu_Hand','Position_hand'); xlabel('t/s');ylabel('x/m and F/mN'); 
subplot(312)
plot(chabu_Hand(:,1),chabu_Hand(:,3)-chabu_Hand(1,3),'b-');hold on;
plot(Position_hand(:,1),Position_hand(:,3)-Position_hand(1,3),'g-');hold on;
legend('chabu_Hand','Position_hand'); xlabel('t/s');ylabel('x/m and F/mN'); 
subplot(313)
plot(chabu_Hand(:,1),chabu_Hand(:,4)-chabu_Hand(1,4),'b-');hold on;
plot(Position_hand(:,1),Position_hand(:,4)-Position_hand(1,4),'g-');hold on;
legend('chabu_Hand','Position_hand'); xlabel('t/s');ylabel('x/m and F/mN'); 
suptitle('chabu_Hand');

%compare the data before and after interp
figure
subplot(311)
plot(chabu_Force(:,1),chabu_Force(:,2)-chabu_Force(1,2),'b-');hold on;
plot(Force(:,1),Force(:,2)-Force(1,2),'g-');hold on;
legend('chabu_Force','Force'); xlabel('t/s');ylabel('x/m and F/mN'); 
subplot(312)
plot(chabu_Force(:,1),chabu_Force(:,3)-chabu_Force(1,3),'b-');hold on;
plot(Force(:,1),Force(:,3)-Force(1,3),'g-');hold on;
legend('chabu_Force','Force'); xlabel('t/s');ylabel('x/m and F/mN'); 
subplot(313)
plot(chabu_Force(:,1),chabu_Force(:,4)-chabu_Force(1,4),'b-');hold on;
plot(Force(:,1),Force(:,4)-Force(1,4),'g-');hold on;
legend('chabu_Force','Force'); xlabel('t/s');ylabel('x/m and F/mN'); 
suptitle('chabu_Force');

pause
close all;
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 3.6.1 transfer force and velocity from sensor to knife
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
world_T_ft =[-0.0032, 1.0000, 0.0062, 0.0527;
    -1.0000, -0.0032, -0.0090, 0.9641;
    -0.0090, -0.0062, 0.9999, 0.3465;
    0, 0, 0, 1];
  
  ft_T_world = [-0.0032   -1.0000    0.0090    0.9611;
 1.0000   -0.0032    0.0062   -0.0518;
-0.0062    0.0090    0.9999   -0.3547;
      0         0         0    1.0000;]

for i=1:length(chabu_Hand(:,1))
% Convert quaternion to rotation matrix
rotm=quat2rotm([chabu_Hand(i,8) chabu_Hand(i,5:7)]);

world_T_knife=[rotm chabu_Hand(i,2:4)';0 0 0 1];

ft_T_knife=ft_T_world*world_T_knife;

knife_T_ft=inv(ft_T_knife);

F_matrix_knife=knife_T_ft*[chabu_Force(i,2:4) 1]';
F_knife(i,2:4)=F_matrix_knife(1:3);
F_knife(i,1)=chabu_Force(i,1);
end

   


%% check force result
figure
subplot(311)
plot(F_knife(:,1),F_knife(:,2)-F_knife(1,2),'b-');hold on;
plot(chabu_Force(:,1),chabu_Force(:,2)-chabu_Force(1,2),'g-');hold on;
legend('F_knife','chabu_Force'); xlabel('t/s');ylabel('x/m and F/mN'); 
subplot(312)
plot(F_knife(:,1),F_knife(:,3)-F_knife(1,3),'b-');hold on;
plot(chabu_Force(:,1),chabu_Force(:,3)-chabu_Force(1,3),'g-');hold on;
legend('F_knife','chabu_Force'); xlabel('t/s');ylabel('x/m and F/mN'); 
subplot(313)
plot(F_knife(:,1),F_knife(:,4)-F_knife(1,4),'b-');hold on;
plot(chabu_Force(:,1),chabu_Force(:,4)-chabu_Force(1,4),'g-');hold on;
legend('F_knife','chabu_Force'); xlabel('t/s');ylabel('x/m and F/mN'); 
suptitle('F_knife with raw force');

figure
subplot(311)
plot(F_knife(:,1),F_knife(:,2)-F_knife(1,2),'b-');hold on;
legend('F_knife'); xlabel('t/s');ylabel('x/m and F/mN'); 
subplot(312)
plot(F_knife(:,1),F_knife(:,3)-F_knife(1,3),'b-');hold on;
legend('F_knife'); xlabel('t/s');ylabel('x/m and F/mN'); 
subplot(313)
plot(F_knife(:,1),F_knife(:,4)-F_knife(1,4),'b-');hold on;
legend('F_knife'); xlabel('t/s');ylabel('x/m and F/mN'); 
suptitle('F_knife');

pause;



%% %%%%%%%%%%%%%%%%
%% 3.6.2 calculate imp: here is only a simple estimate of impedance
%%                    using EMG and a fixed arm pose
%% %%%%%%%%%%%%%%%%%%
angle=[];
for i=1:length(chabu_EMG)
    co_contravc_EMG_plot(i,1)=chabu_EMG(i,1);
    co_contravc_EMG_plot(i,2)=min(chabu_EMG(i,4:7));
end

co_contravc_EMG=co_contravc_EMG_plot;


    GAIN = 2.112983215e+06;  %1.112983215e+06;

    A=[0.0197241514182339;0.0131874934229002;0.0140010076275596]*2;
    B=[15;7;7];


    xv=zeros(5,1);
    yv=zeros(5,1);
    for i=1:length(co_contravc_EMG)
        % filter EMG
        xv(1) = xv(2); xv(2) = xv(3); xv(3) = xv(4); xv(4) = xv(5); 
        xv(5) = co_contravc_EMG(i,2) / GAIN;
        yv(1) = yv(2); yv(2) = yv(3); yv(3) = yv(4); yv(4) = yv(5); 
        yv(5) = (xv(1) + xv(5)) + 4 * (xv(2) + xv(4)) + 6 * xv(3)+ ( -0.8485559993 * yv(1)) + (  3.5335352195 * yv(2))+ ( -5.5208191366 * yv(3)) + (  3.8358255406 * yv(4));
        EMG_filter = yv(5);

        L(1)=A(1)*EMG_filter+B(1);
        L(2)=A(2)*EMG_filter+B(2);
        L(3)=A(3)*EMG_filter+B(3);
        L=L';

%     %%%%% if calcualte angle
%         Alpha_mat=asin((abs(chabu_Hand(i,3)-chabu_Elbow(i,3)))/sqrt(power(chabu_Hand(i,2)-chabu_Elbow(i,2),2)+power(chabu_Hand(i,3)-chabu_Elbow(i,3),2)));
%         Beta_mat=asin((abs(chabu_Hand(i,4)-chabu_Elbow(i,4)))/sqrt(power(chabu_Hand(i,4)-chabu_Elbow(i,4),2)+power(chabu_Hand(i,2)-chabu_Elbow(i,2),2)+power(chabu_Hand(i,3)-chabu_Elbow(i,3),2)));
        
    %%%%%% define a good angle
        Alpha_mat=0.54;
        Beta_mat=0.02;
        
        aaa=[Alpha_mat;Beta_mat];
        angle=[angle aaa];

        COS_fun=[cos(Beta_mat)*cos(Alpha_mat)   cos(Beta_mat)*sin(Alpha_mat)   sin(Beta_mat);
                    sin(Alpha_mat)           cos(Alpha_mat)       0.00;
                  sin(Beta_mat)*cos(Alpha_mat)   sin(Beta_mat)*sin(Alpha_mat)   cos(Alpha_mat);];

        L=COS_fun*L;

        IMP(i,1)=co_contravc_EMG(i,1);
        IMP(i,2)=power(L(1),2);
        IMP(i,3)=power(L(2),2);
        IMP(i,4)=power(L(3),2);

        clear L COS_fun

    end
    
    
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 3.7 process position and get velocity and eigenvector 1 for passive DS
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
%% using opt get the eigvector of motion
frequence_time=(F_knife(end,3)-F_knife(1,3))/length(F_knife(:,3));
n=1;

% Do filtering and generate velocity
dt             = frequence_time;
window_size    = 21;
crop_size = (window_size+1)/2;

dx_nth         = sgolay_time_derivatives(chabu_Hand(1:end,2:4), dt, 2, 3, window_size);
chabu_Hand1(:,2:4) = dx_nth(:,:,1);
chabu_Hand1(:,5:7) = dx_nth(:,:,2); 
    clear dx_nth
    chabu_Hand1=downsample(chabu_Hand1, n);chabu_Hand1=chabu_Hand1(:,2:7);
    
dx_nth         = sgolay_time_derivatives(chabu_Hand(1:end,5:7), dt, 2, 3, window_size);
Ori_hand(:,1:3) = dx_nth(:,:,1);
    clear dx_nth
dx_nth         = sgolay_time_derivatives(chabu_Hand(1:end,6:8), dt, 2, 3, window_size);
Ori_hand(:,4:6) = dx_nth(:,:,1);
    clear dx_nth
    Ori_hand=downsample(Ori_hand, n);
    sum_ori1=sum(Ori_hand(:,4)-Ori_hand(:,2));
    sum_ori2=sum(Ori_hand(:,5)-Ori_hand(:,3));
    if sum_ori1==0 & sum_ori2==0
    else
        !echo Ori Error
        break
    end

dx_nth         = sgolay_time_derivatives(IMP(1:end,2:4), dt, 2, 3, window_size);
IMP1(:,2:4) = dx_nth(:,:,1);
IMP1(:,5:7) = dx_nth(:,:,2); 
   clear dx_nth 
   IMP1=downsample(IMP1, n);IMP1=IMP1(:,2:7);

dx_nth         = sgolay_time_derivatives(F_knife(1:end,2:4), dt, 2, 3, window_size);
F_knife1(:,2:4) = dx_nth(:,:,1);
F_knife1(:,5:7) = dx_nth(:,:,2); 
    clear dx_nth 
    F_knife1=downsample(F_knife1, n);F_knife1=F_knife1(:,2:7);


    %% velocity in kinfe frame
for i=1:length(Ori_hand(:,1))
    % Convert quaternion to rotation matrix
    rotm=quat2rotm([Ori_hand(i,6) Ori_hand(i,1:3)]);

    world_T_knife=[rotm chabu_Hand1(i,1:3)';0 0 0 1];

    ft_T_knife=ft_T_world*world_T_knife;

    knife_T_world=inv(world_T_knife);

    V_matrix_knife=knife_T_world*[chabu_Hand1(i,4:6) 1]';
    V_knife(i,1:3)=V_matrix_knife(1:3);
end

dx_nth         = sgolay_time_derivatives(V_knife(1:end,1:3), dt, 2, 3, window_size);
V_knife1(:,1:3) = dx_nth(:,:,1);
V_knife1(:,4:6) = dx_nth(:,:,2); 
    clear dx_nth 
    V_knife1=downsample(V_knife1, n);
        
%% plot pos and vel
if plot_all_figure==1
   figure
   subplot(3,2,1);
    plot(1:length(chabu_Hand1(:,1)),chabu_Hand1(:,1));hold on; grid on;
    subplot(3,2,2);
    plot(1:length(chabu_Hand1(:,1)),chabu_Hand1(:,4));hold on; grid on;
    subplot(3,2,3);
    plot(1:length(chabu_Hand1(:,1)),chabu_Hand1(:,2));hold on; grid on;
    subplot(3,2,4);
    plot(1:length(chabu_Hand1(:,1)),chabu_Hand1(:,5));hold on; grid on;
    subplot(3,2,5);
    plot(1:length(chabu_Hand1(:,1)),chabu_Hand1(:,3));hold on; grid on;
    subplot(3,2,6);
    plot(1:length(chabu_Hand1(:,1)),chabu_Hand1(:,6));hold on; grid on;
end
%% vel in 3D trj
    vel_samples = 3; vel_size = 3;
    f1=plot_vel(chabu_Hand1(:,:)',vel_samples,vel_size);
%     subplot(122);
%     plot3(chabu_Hand(:,2),chabu_Hand(:,3),chabu_Hand(:,4),'b-','linewidth',3);hold on; grid on;
    

%% get the eigen_vector1
    eigen_vector1=[];
    for iopt1=1:length(chabu_Hand1(:,1))
        eigen_vector1=[eigen_vector1; chabu_Hand1(iopt1,4:6)/norm(chabu_Hand1(iopt1,4:6))];
    end
    
   if plot_all_figure==1
   figure
   subplot(3,2,1);
    plot(1:length(eigen_vector1(:,1)),eigen_vector1(:,1));hold on; grid on;
    subplot(3,2,3);
    plot(1:length(eigen_vector1(:,1)),eigen_vector1(:,2));hold on; grid on;
    subplot(3,2,5);
    plot(1:length(eigen_vector1(:,1)),eigen_vector1(:,3));hold on; grid on;
    
     pause
   end



%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 3.7 plot all result
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
if plot_all_figure==1
    
    h1=figure;a=plot_wr(h1,2);
    angle=angle';
    subplot(211)
    plot(1:length(IMP1(:,1)),IMP1(:,1) ,'LineWidth',3);hold on;
    plot(1:length(IMP1(:,1)),IMP1(:,2) ,'LineWidth',3);hold on;
    plot(1:length(IMP1(:,1)),IMP1(:,3) ,'LineWidth',3);hold on;
    legend('IMPX','IMPY','IMPZ');xlabel('t/s');ylabel('m');title('I');
    subplot(212)
    plot(1:length(IMP1(:,1)),IMP1(:,4) ,'LineWidth',3);hold on;
    plot(1:length(IMP1(:,1)),IMP1(:,5) ,'LineWidth',3);hold on;
    plot(1:length(IMP1(:,1)),IMP1(:,6) ,'LineWidth',3);hold on;
    legend('d-IMPX','d-IMPY','d-IMPZ');xlabel('t/s');ylabel('m');title('dI');
%     picture2Dsavename=[path_of_plot 'imp_angle'];
%     saveas(h1,[picture2Dsavename '.fig'])
    title('IMP_knife');


    h1=figure;a=plot_wr(h1,2);
    angle=angle';
    subplot(211)
    plot(1:length(F_knife1(:,1)),F_knife1(:,1) ,'LineWidth',3);hold on;
    plot(1:length(F_knife1(:,1)),F_knife1(:,2) ,'LineWidth',3);hold on;
    plot(1:length(F_knife1(:,1)),F_knife1(:,3) ,'LineWidth',3);hold on;
    legend('F_knife_X','F_knife_Y','F_knife_Z');xlabel('t/s');ylabel('m');title('F');
    subplot(212)
    plot(1:length(F_knife1(:,1)),F_knife1(:,4) ,'LineWidth',3);hold on;
    plot(1:length(F_knife1(:,1)),F_knife1(:,5) ,'LineWidth',3);hold on;
    plot(1:length(F_knife1(:,1)),F_knife1(:,6) ,'LineWidth',3);hold on;
    legend('dF_knife_X','dF_knife_Y','dF_knife_Z');xlabel('t/s');ylabel('m');title('dF');
%     picture2Dsavename=[path_of_plot 'imp_angle'];
%     saveas(h1,[picture2Dsavename '.fig'])
    suptitle('F_knife');

    h1=figure;a=plot_wr(h1,2);
    angle=angle';
    subplot(211)
    plot(1:length(V_knife1(:,1)),V_knife1(:,1) ,'LineWidth',3);hold on;
    plot(1:length(V_knife1(:,1)),V_knife1(:,2) ,'LineWidth',3);hold on;
    plot(1:length(V_knife1(:,1)),V_knife1(:,3) ,'LineWidth',3);hold on;
    legend('V_knife_X','V_knife_Y','V_knife_Z');xlabel('t/s');ylabel('m');title('V');
    subplot(212)
    plot(1:length(V_knife1(:,1)),V_knife1(:,4) ,'LineWidth',3);hold on;
    plot(1:length(V_knife1(:,1)),V_knife1(:,5) ,'LineWidth',3);hold on;
    plot(1:length(V_knife1(:,1)),V_knife1(:,6) ,'LineWidth',3);hold on;
    legend('dV_knife_X','dV_knife_Y','dV_knife_Z');xlabel('t/s');ylabel('m');title('A');
%     picture2Dsavename=[path_of_plot 'imp_angle'];
%     saveas(h1,[picture2Dsavename '.fig'])
    suptitle('V_knife');
    
    h1=figure;a=plot_wr(h1,2);
    angle=angle';
    subplot(211)
    plot(1:length(chabu_Hand1(:,1)),chabu_Hand1(:,1) ,'LineWidth',3);hold on;
    plot(1:length(chabu_Hand1(:,1)),chabu_Hand1(:,2) ,'LineWidth',3);hold on;
    plot(1:length(chabu_Hand1(:,1)),chabu_Hand1(:,3) ,'LineWidth',3);hold on;
    legend('P_X','P_Y','P_Z');xlabel('t/s');ylabel('m');title('P');
    subplot(212)
    plot(1:length(chabu_Hand1(:,1)),chabu_Hand1(:,4) ,'LineWidth',3);hold on;
    plot(1:length(chabu_Hand1(:,1)),chabu_Hand1(:,5) ,'LineWidth',3);hold on;
    plot(1:length(chabu_Hand1(:,1)),chabu_Hand1(:,6) ,'LineWidth',3);hold on;
    legend('dP_X','dP_Y','dP_Z');xlabel('t/s');ylabel('m');title('V');
%     picture2Dsavename=[path_of_plot 'imp_angle'];
%     saveas(h1,[picture2Dsavename '.fig'])
    suptitle('demo trj and vel');
    
    pause
    
end


   
   

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 4 save data
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
filename1=[path_of_save  task '_' exp_name '_' exp_time  '.mat'];   
    save(filename1, 'IMP1','F_knife1','chabu_Hand1','eigen_vector1','V_knife1')
    !echo save done

end

