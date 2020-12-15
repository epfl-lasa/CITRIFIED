%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Rui Wu 2020.12.15
%   process all data and get impedance estimate
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i=1:10
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
% task='avocado';
% task='banana';
task='orange';

%%%%% auto
% exp_name=['deep'];
% exp_name=['good'];
exp_name=['shallow'];

% exp_time=['1'];
% exp_time=[num2str(str2num(exp_time)+1)]
exp_time=[num2str(i)]

plot_all_figure=0;


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
formatSpec = '%f%f%f%f%f%f%f%f%f%*s%*s%*s%*s%*s%*s%*s%*s%[^\n\r]';
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
%% Force
Force=zeros(length(ftsensor),4);
for i=1:length(ftsensor)  
    Force(i,1)=ftsensor(i,3);%+0.01;%+0.01
    Force(i,2)=ftsensor(i,4);%/1000; 
    Force(i,3)=ftsensor(i,5);%/1000;
    Force(i,4)=ftsensor(i,6);%/1000;
end

Force(:,2:4)=Force(:,2:4)-Force(1,2:4); 

%% transfer force from sensor to base
TransF=[-0.0034    1.0000    0.0063   -0.3649;
-1.0000   -0.0034   -0.0088    0.1294;
-0.0088   -0.0063    0.9999   -0.6664;
      0         0         0    1.0000;];
  
TransF_4=[];
for fi=1:length(Force(:,4))
  TransF_4=[TransF_4 TransF*[Force(fi,2:4) 1]'];
end
Force(:,2:4)=TransF_4(1:3,:)';

%% opt data
Position_hand=zeros(length(opt),4);
for i=1:length(opt)  
    Position_hand(i,1)=opt(i,3);
    Position_hand(i,2)=opt(i,4);
    Position_hand(i,3)=opt(i,5);
    Position_hand(i,4)=opt(i,6);
end 

Position_elbow=zeros(length(opt),4);
for i=1:length(opt)  
    Position_elbow(i,1)=opt(i,3);
    Position_elbow(i,2)=opt(i,7);
    Position_elbow(i,3)=opt(i,8);
    Position_elbow(i,4)=opt(i,9);
end 

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

chabu_Hand=ALL_data(:,1:4);
chabu_Elbow=ALL_data(:,5:8);
chabu_Force=ALL_data(:,9:12);
chabu_EMG=ALL_data(:,13:end);

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


%% %%%%%%%%%%%%%%%%
%% 3.6 calculate imp: here is only a simple estimate of impedance
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
    
    
if plot_all_figure==1
    h1=figure;a=plot_wr(h1,2);
    angle=angle';
%     subplot(211)
    plot(IMP(:,1),IMP(:,2) ,'LineWidth',3);hold on;
    plot(IMP(:,1),IMP(:,3) ,'LineWidth',3);hold on;
    plot(IMP(:,1),IMP(:,4) ,'LineWidth',3);hold on;
    legend('IMPX','IMPY','IMPZ');xlabel('t/s');ylabel('m');title('IMP');
%     subplot(212)
%     plot(IMP(:,1),angle(:,1));hold on;
%     plot(IMP(:,1),angle(:,2));hold on;
%     legend('alpha','beta');
%     picture2Dsavename=[path_of_plot 'imp_angle'];
%     saveas(h1,[picture2Dsavename '.fig'])
    
    
    figure
    for j=1:300:length(co_contravc_EMG)
    plot3([chabu_Hand(j,2) chabu_Elbow(j,2)],[chabu_Hand(j,3) chabu_Elbow(j,3)],[chabu_Hand(j,4) chabu_Elbow(j,4)]);hold on;grid on;
    end
    plot3(chabu_Hand(1:end,2),chabu_Hand(1:end,3),chabu_Hand(1:end,4),'r');hold on;grid on;
    plot3(chabu_Hand(1,2),chabu_Hand(1,3),chabu_Hand(1,4),'ro');hold on;grid on;
    plot3(chabu_Hand(end,2),chabu_Hand(end,3),chabu_Hand(end,4),'r*');hold on;grid on;
    plot3(chabu_Elbow(1:end,2),chabu_Elbow(1:end,3),chabu_Elbow(1:end,4),'b');hold on;grid on;
    plot3(chabu_Elbow(1,2),chabu_Elbow(1,3),chabu_Elbow(1,4),'bo');hold on;grid on;
    plot3(chabu_Elbow(end,2),chabu_Elbow(end,3),chabu_Elbow(end,4),'b*');hold on;grid on;
     axis equal;
    xlabel('x');ylabel('y');zlabel('z');
    legend('hand','elbow');title('hand');
    length(co_contravc_EMG)
    
     h2=figure;a=plot_wr(h2,3);
subplot(331);plot(chabu_EMG(:,1),chabu_EMG(:,2),'b-');
        hold on;plot(EMG(:,1),EMG(:,2),'r-','linewidth',2);
subplot(332);plot(chabu_EMG(:,1),chabu_EMG(:,3),'b-');
        hold on;plot(EMG(:,1),EMG(:,3),'r-','linewidth',2);
subplot(333);plot(chabu_EMG(:,1),chabu_EMG(:,4),'b-');
        hold on;plot(EMG(:,1),EMG(:,4),'r-','linewidth',2);
subplot(334);plot(chabu_EMG(:,1),chabu_EMG(:,5),'b-');
        hold on;plot(EMG(:,1),EMG(:,5),'r-','linewidth',2);
subplot(335);plot(chabu_EMG(:,1),chabu_EMG(:,6),'b-');
        hold on;plot(EMG(:,1),EMG(:,6),'r-','linewidth',2);
subplot(336);plot(chabu_EMG(:,1),chabu_EMG(:,7),'b-');
        hold on;plot(EMG(:,1),EMG(:,7),'r-','linewidth',2);
subplot(337);plot(chabu_EMG(:,1),chabu_EMG(:,8),'b-');
        hold on;plot(EMG(:,1),EMG(:,8),'r-','linewidth',2);
subplot(338);plot(chabu_EMG(:,1),chabu_EMG(:,9),'b-');
       hold on;plot(EMG(:,1),EMG(:,9),'r-','linewidth',2);
subplot(339);plot(chabu_EMG(:,1),chabu_EMG(:,10),'b-');
        hold on;plot(EMG(:,1),EMG(:,10),'r-','linewidth',2);
legend('EMG');xlabel('t/s');ylabel('amplitute');suptitle('EMG');
%         picture2Dsavename=[path_of_plot 'EMG'];
%     saveas(h2,[picture2Dsavename '.fig'])
        
            h3=figure;a=plot_wr(h3,4);
subplot(331);plot(chabu_Elbow(:,1),chabu_Elbow(:,2),'b-','linewidth',2);
        hold on;plot(Position_elbow(:,1),Position_elbow(:,2),'r-','linewidth',1);
subplot(332);plot(chabu_Elbow(:,1),chabu_Elbow(:,3),'b-','linewidth',2);
        hold on;plot(Position_elbow(:,1),Position_elbow(:,3),'r-','linewidth',1);
subplot(333);plot(chabu_Elbow(:,1),chabu_Elbow(:,4),'b-','linewidth',2);
        hold on;plot(Position_elbow(:,1),Position_elbow(:,4),'r-','linewidth',1);
subplot(334);plot(chabu_Hand(:,1),chabu_Hand(:,2),'b-','linewidth',2);
        hold on;plot(Position_hand(:,1),Position_hand(:,2),'r-','linewidth',1);
subplot(335);plot(chabu_Hand(:,1),chabu_Hand(:,3),'b-','linewidth',2);
        hold on;plot(Position_hand(:,1),Position_hand(:,3),'r-','linewidth',1);
subplot(336);plot(chabu_Hand(:,1),chabu_Hand(:,4),'b-','linewidth',2);
        hold on;plot(Position_hand(:,1),Position_hand(:,4),'r-','linewidth',1);
subplot(337);plot(chabu_Force(:,1),chabu_Force(:,2),'b-','linewidth',2);
        hold on;plot(Force(:,1),Force(:,2),'r-','linewidth',1);
subplot(338);plot(chabu_Force(:,1),chabu_Force(:,3),'b-','linewidth',2);
       hold on;plot(Force(:,1),Force(:,3),'r-','linewidth',1);
subplot(339);plot(chabu_Force(:,1),chabu_Force(:,4),'b-','linewidth',2);
        hold on;plot(Force(:,1),Force(:,4),'r-','linewidth',1);
        suptitle('1 Wrster 2 tool 3 Force');
%     picture2Dsavename=[path_of_plot 'position_and_force'];
%     saveas(h3,[picture2Dsavename '.fig'])
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 3.7 process position and get velocity and eigenvector 1 for passive DS
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
%% using opt get the eigvector of motion
frequence_time=(Force(end,3)-Force(1,3))/length(Force(:,3));
n=4;

    % Do filtering and generate velocity
    dt             = frequence_time;
    window_size    = 21;
    crop_size = (window_size+1)/2;
    
    dx_nth         = sgolay_time_derivatives(chabu_Hand(1:end,2:4), dt, 2, 3, window_size);
    chabu_Hand1(:,2:4) = dx_nth(:,:,1);
    chabu_Hand1(:,5:7) = dx_nth(:,:,2); 
        clear dx_nth
    dx_nth         = sgolay_time_derivatives(IMP(1:end,2:4), dt, 2, 3, window_size);
    IMP1(:,2:4) = dx_nth(:,:,1);
    IMP1(:,5:7) = dx_nth(:,:,2); 
       clear dx_nth 
    dx_nth         = sgolay_time_derivatives(chabu_Force(1:end,2:4), dt, 2, 3, window_size);
    chabu_Force1(:,2:4) = dx_nth(:,:,1);
    chabu_Force1(:,5:7) = dx_nth(:,:,2); 
        clear dx_nth 
    
    chabu_Hand1=downsample(chabu_Hand1, n);
    IMP1=downsample(IMP1, n);
    chabu_Force1=downsample(chabu_Force1, n);
    
    IMP1=IMP1(:,2:7);chabu_Force1=chabu_Force1(:,2:7);chabu_Hand1=chabu_Hand1(:,2:7);
        
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
%% 3.2 process force (wait for the force trans to knife coordinate)
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% get force follow the knife tip

% for i=1:length(chabu_Force1(:,1))
%     chabu_Force1(i,1:3)=chabu_Force1(i,1:3)'*eigen_vector1(i,1:3)
% end


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 4 save data
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% filename1=[path_of_save  task '_' exp_name '_' exp_time  '.mat'];   
%     save(filename1, 'IMP1','chabu_Force1','chabu_Hand1','eigen_vector1')
%     !echo save done

end


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% all function
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ P16, raw_EMG_flitered] = EMG_process_9ch( approximatetime_data_EMG )
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
pt=[approximatetime_data_EMG(:,2)-mean(approximatetime_data_EMG(:,2))...
    approximatetime_data_EMG(:,3)-mean(approximatetime_data_EMG(:,3))...
    approximatetime_data_EMG(:,4)-mean(approximatetime_data_EMG(:,4))...
    approximatetime_data_EMG(:,5)-mean(approximatetime_data_EMG(:,5))...
    approximatetime_data_EMG(:,6)-mean(approximatetime_data_EMG(:,6))...
    approximatetime_data_EMG(:,7)-mean(approximatetime_data_EMG(:,7))...
    approximatetime_data_EMG(:,8)-mean(approximatetime_data_EMG(:,8))...
    approximatetime_data_EMG(:,9)-mean(approximatetime_data_EMG(:,9))...
    approximatetime_data_EMG(:,10)-mean(approximatetime_data_EMG(:,10))];

time = approximatetime_data_EMG(:,1);
for chanl = 1:(length(approximatetime_data_EMG(1,:))-1)
    ch1=pt(:,chanl);
    % define sampling rate (acquisition frequency)
    SR = 1000;

    % define a time window for extracting the features
    tw = 100; % ms

    % let's use only a band-pass filter on the EMG signals
    % define a small time-window as a buffer for the filters to converge
    s_buffer = 30; % ms

    % cut-off frequencies of the band-pass filter 
    bandPassCuttOffFreq=[50,400]; % Hz
    % bandPassCuttOffFreq=[1,8]; % Hz

    % the order of the filter
    filt_order = 4;

    % let's extract 3 features from the time-window: mean-average, its standard
    % deviation and the number of zero-crossings

    % define the feature vector (check the documentation of the function
    % "exctractFeatures")
    featuresIDs = [0, 1, 0, 1, 0, 1, 0];

    % find the actual number of samples that correspond to the time-window and
    % the buffer
    tw_samples = tw * SR / 1000;

    buffer_samples = s_buffer * SR / 1000;

    % compute the filter coefficients for the band-pass butterworth filter

    Wn=(bandPassCuttOffFreq(1)*2)/SR;

    [B_H, A_H] = butter(filt_order, Wn, 'high'); 

    Wn=(bandPassCuttOffFreq(2)*2)/SR;

    [B_L, A_L] = butter(filt_order, Wn, 'low'); 

    % pre-process the data

    resultedSignal = [];
    tw_features = [];
    tw_labels = [];
    buffer = [];
    
    %%%%%%%%%%%%% for matlab offline %%%%%%%%%%%
    filteredSignal_new = preprocEMG(ch1,SR, B_H, A_H, B_L, A_L, 0, 0, false, false, [], 0 , false);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%% for real time %%%%%%%%%%%%%%%%%%
    for i=1:tw_samples:length(ch1)
        if i+tw < length(ch1)
            tWindow = ch1(i:i+tw_samples - 1 );
    %         tLabels = newLabels(i:i+tw_samples - 1);
        else
            tWindow = ch1(i:end); % end 
    %         tLabels = newLabels(i:end);
        end

        if i ~= 1
            tWindow = [buffer;tWindow];
        end
        if i+tw < length(ch1)
            buffer = tWindow(end - buffer_samples +1 :end);
        end
        if i==1
            filteredSignal = preprocEMG(tWindow,SR, B_H, A_H, B_L, A_L, 0, 0, false, false, [], 0 , false);
        else
            filteredSignal = preprocEMG(tWindow,SR, B_H, A_H, B_L, A_L, 0, 0, false, false, [], s_buffer , false);
        end
        resultedSignal = [resultedSignal; filteredSignal];

        tw_features = [tw_features; exctractFeatures(filteredSignal, featuresIDs)];

    %     tw_labels = [tw_labels;round(mean(tLabels))];
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
%     %%%%%%%%%%%%%%  plot
%     % inspect the data
%     figure()
%     plot(time, ch1)
%     hold on
%     plot(time, resultedSignal)
%     plot(time, filteredSignal_new)
%     % plot(time, newLabels*1000)
%     legend('original signal', 'filtered signal', 'new filtered signal')
%     %%%%%%%%%%%%%%  plot
    
    pt(:,chanl)=filteredSignal_new;
    
end
raw_EMG_flitered=[approximatetime_data_EMG(:,1) pt];

%% my way

k1=1;k2=length(approximatetime_data_EMG(:,3));
for i=k1:k2
    j=i-k1+1;
    Pa(j,1)=pt(i,1);
    Pb(j,1)=pt(i,2);
    Pc(j,1)=pt(i,3);
    Pd(j,1)=pt(i,4);
    Pe(j,1)=pt(i,5);
    Pf(j,1)=pt(i,6);
    Pg(j,1)=pt(i,7);
    Ph(j,1)=pt(i,8);
    Pi(j,1)=pt(i,9);
end
fs=1000;              %锟斤拷锟斤拷频锟斤拷  
N=100;                %锟斤拷锟节筹拷锟斤拷 
L=length(Pa); 
n=0:L-1;              
t=n/fs;             %时锟斤拷
a=mod(L,N);
for j=0:N:L-a
     if j==L-a
       for i=1:a
       ma(i)=abs(Pa(i+j));
       mb(i)=abs(Pb(i+j));
       mc(i)=abs(Pc(i+j));
       md(i)=abs(Pd(i+j));
       me(i)=abs(Pe(i+j));
       mf(i)=abs(Pf(i+j));
       mg(i)=abs(Pg(i+j));
       mh(i)=abs(Ph(i+j));
       mi(i)=abs(Pi(i+j));
       end
       for i=1:a
        P1(i+j)=sum(ma)/N;
        P2(i+j)=sum(mb)/N;
        P3(i+j)=sum(mc)/N;
        P4(i+j)=sum(md)/N;
        P5(i+j)=sum(me)/N;
        P6(i+j)=sum(mf)/N;
        P7(i+j)=sum(mg)/N;
        P8(i+j)=sum(mh)/N;
        P9(i+j)=sum(mi)/N;
       end 
    else
for i=1:N
ma(i)=abs(Pa(i+j));
mb(i)=abs(Pb(i+j));
mc(i)=abs(Pc(i+j));
md(i)=abs(Pd(i+j));
me(i)=abs(Pe(i+j));
mf(i)=abs(Pf(i+j));
mg(i)=abs(Pg(i+j));
mh(i)=abs(Ph(i+j));
mi(i)=abs(Pi(i+j));
end
for i=1:N
P1(i+j)=sum(ma)/N;
P2(i+j)=sum(mb)/N;
P3(i+j)=sum(mc)/N;
P4(i+j)=sum(md)/N;
P5(i+j)=sum(me)/N;
P6(i+j)=sum(mf)/N;
P7(i+j)=sum(mg)/N;
P8(i+j)=sum(mh)/N;
P9(i+j)=sum(mi)/N;
end
    end
end
P1=P1';
P2=P2';
P3=P3';
P4=P4';
P5=P5';
P6=P6';
P7=P7';
P8=P8';
P9=P9';

% plot(t,P(:,1));hold on;

% plot(t,P1(:,1));
% k1=5000;k2=10000;
% for i=k1:k2
%     j=i-k1+1;
%     Paa(j,1)=P1(i,1);
%     Pbb(j,1)=P2(i,1);
%     Pcc(j,1)=P3(i,1);
%     Pdd(j,1)=P4(i,1);
%     Pee(j,1)=P5(i,1);
%     Pff(j,1)=P6(i,1);
% end
% Paf=[Paa Pbb Pcc Pdd Pee Pff];
%  Pa_ave=mean(Paa);
%  Pb_ave=mean(Pbb);
%  Pc_ave=mean(Pcc);
%  Pd_ave=mean(Pdd);
%  Pe_ave=mean(Pee);
%  Pf_ave=mean(Pff);
 
 P1=smooth(P1,200);
  P2=smooth(P2,200);
   P3=smooth(P3,200);
    P4=smooth(P4,200);
     P6=smooth(P6,200);
      P5=smooth(P5,200);
      P7=smooth(P7,200);
      P8=smooth(P8,200);
      P9=smooth(P9,200);
      
 P16=[approximatetime_data_EMG(:,1) P1 P2 P3 P4 P5 P6 P7 P8 P9];
 
f=figure;a=plot_wr(f,1);
subplot(331);plot(approximatetime_data_EMG(:,1),pt(:,1),'b-');
        hold on;plot(P16(:,1),P16(:,2),'r-','linewidth',2);
subplot(332);plot(approximatetime_data_EMG(:,1),pt(:,2),'b-');
        hold on;plot(P16(:,1),P16(:,3),'r-','linewidth',2);
subplot(333);plot(approximatetime_data_EMG(:,1),pt(:,3),'b-');
        hold on;plot(P16(:,1),P16(:,4),'r-','linewidth',2);
subplot(334);plot(approximatetime_data_EMG(:,1),pt(:,4),'b-');
        hold on;plot(P16(:,1),P16(:,5),'r-','linewidth',2);
subplot(335);plot(approximatetime_data_EMG(:,1),pt(:,5),'b-');
        hold on;plot(P16(:,1),P16(:,6),'r-','linewidth',2);
subplot(336);plot(approximatetime_data_EMG(:,1),pt(:,6),'b-');
        hold on;plot(P16(:,1),P16(:,7),'r-','linewidth',2);
subplot(337);plot(approximatetime_data_EMG(:,1),pt(:,7),'b-');
        hold on;plot(P16(:,1),P16(:,8),'r-','linewidth',2);
subplot(338);plot(approximatetime_data_EMG(:,1),pt(:,8),'b-');
       hold on;plot(P16(:,1),P16(:,9),'r-','linewidth',2);
subplot(339);plot(approximatetime_data_EMG(:,1),pt(:,9),'b-');
        hold on;plot(P16(:,1),P16(:,10),'r-','linewidth',2);

end


function [getxX getxY] = ChoicePointByMouse()   

getxX = []; getxY = [];
    while 0<1
        [x,y,b] = ginput(1); 
        if isempty(b); 
            break;
        elseif b==91;
            ax = axis; width=ax(2)-ax(1); height=ax(4)-ax(3);%
            axis([x-width/2 x+width/2 y-height/2 y+height/2]);%y-height/2 y+height/2
            zoom xon;
            zoom(1/2);
        elseif b==93;
            ax = axis; width=ax(2)-ax(1); height=ax(4)-ax(3);
            axis([x-width/2 x+width/2 y-height/2 y+height/2]);
            zoom xon;
            zoom(2);    
        else
            getxX=[getxX;x];
            getxY=[getxY;y];
        end;
    end
    [getxX getxY]

    %% deal data after cut    
%     idf= Force(:,1)<time_begin | Force(:,1)> time_end;
%     Force(idf,:)=[];
%     idp= Pose(:,1)<time_begin | Pose(:,1)> time_end;
%     Pose(idp,:)=[];
    
    
end

function [ NDI ] = Position_filter(NDI,FilterOrNot,filter_kind,low_cut_off,high_cut_off,plot_filter )
%POSITION_FILTER 此处显示有关此函数的摘要
%   此处显示详细说明
%    filter_kind 0 low pass,1 band pass
%    plot_filter=0; 
Position_origh=NDI;

if FilterOrNot==1
    !echo Position is filtering!
    % calculate the fs
    fs=round((length(NDI(:,1))-1)/(NDI(end,1)-NDI(1,1)));%Hz
    %for directly filter, the data should begin form zero
    NDI(:,2:4)=NDI(:,2:4)-NDI(1,2:4);
    %before filtering
    if filter_kind==0
        NDI(:,2)=butterworth_filter(filter_kind,plot_filter,3,NDI(:,2),fs,high_cut_off);
        NDI(:,3)=butterworth_filter(filter_kind,plot_filter,3,NDI(:,3),fs,high_cut_off);
        NDI(:,4)=butterworth_filter(filter_kind,plot_filter,3,NDI(:,4),fs,high_cut_off);
    elseif filter_kind==1
        NDI(:,2)=butterworth_filter(filter_kind,plot_filter,3,NDI(:,2),fs,low_cut_off,high_cut_off);
        NDI(:,3)=butterworth_filter(filter_kind,plot_filter,3,NDI(:,3),fs,low_cut_off,high_cut_off);
        NDI(:,4)=butterworth_filter(filter_kind,plot_filter,3,NDI(:,4),fs,low_cut_off,high_cut_off);
    end
        
    NDI=NDI(:,:)+Position_origh(1,:);% cause the position data not like Force_robot begin from 0, so we add them orign value for real position in world
    
    figure
    subplot(311)
    plot(NDI(:,1),NDI(:,2),'b-',Position_origh(:,1),Position_origh(:,2),'r-')
    legend('after filter','before filter'); 
    subplot(312)
    plot(NDI(:,1),NDI(:,3),'b-',Position_origh(:,1),Position_origh(:,3),'r-')
    legend('after filter','before filter'); 
    subplot(313)
    plot(NDI(:,1),NDI(:,4),'b-',Position_origh(:,1),Position_origh(:,4),'r-')
    legend('after filter','before filter'); 
    
    clear Position_robot
else
    !echo Position not filtering!
end

end


function [ F ] = Force_filter(F,FilterOrNot,filter_kind,low_cut_off,high_cut_off,plot_filter )
%POSITION_FILTER 此处显示有关此函数的摘要
%   此处显示详细说明
%    filter_kind 0 low pass,1 band pass
%    plot_filter=0; 

Force_robot=F;
if FilterOrNot==1
     % calculate the fs 
    fs=round((length(F(:,1))-1)/(F(end,1)-F(1,1)));%Hz
    %for directly filter, the data should begin form zero
	F(:,2:4)=F(:,2:4)-F(1,2:4);
    !echo Force is filtering!
    if filter_kind==0
        F(:,2)=butterworth_filter(filter_kind,plot_filter,3,F(:,2),fs,high_cut_off);
        F(:,3)=butterworth_filter(filter_kind,plot_filter,3,F(:,3),fs,high_cut_off);
        F(:,4)=butterworth_filter(filter_kind,plot_filter,3,F(:,4),fs,high_cut_off);
    elseif filter_kind==1
        F(:,2)=butterworth_filter(filter_kind,plot_filter,3,F(:,2),fs,low_cut_off,high_cut_off);
        F(:,3)=butterworth_filter(filter_kind,plot_filter,3,F(:,3),fs,low_cut_off,high_cut_off);
        F(:,4)=butterworth_filter(filter_kind,plot_filter,3,F(:,4),fs,low_cut_off,high_cut_off);
    end
    figure
    subplot(311)
    plot(F(:,1),F(:,2),'b-',Force_robot(:,1),Force_robot(:,2),'r-')
    legend('after filter','before filter'); 
    subplot(312)
    plot(F(:,1),F(:,3),'b-',Force_robot(:,1),Force_robot(:,3),'r-')
    legend('after filter','before filter'); 
    subplot(313)
    plot(F(:,1),F(:,4),'b-',Force_robot(:,1),Force_robot(:,4),'r-')
    legend('after filter','before filter'); 
    
    
    clear Force_robot
else
    !echo Force not filtering!
end

end

function [ output_args ] = plot_wr(figure_obj,position_and_size)
%%% plot 
% this funciton will give each plot a framework to decied size, position,
% front, color, linetype, linewidth, title .......

size=get(0,'ScreenSize');
if position_and_size==1
    figure_obj.Position=[1 1 size(3:4)*0.8];
    
%     set(gcf,'unit','centimeters','position',[0 0 size(3:4)*0.8])
    set(gca,'Position',[.15 .15 .8 .75],'FontName','Times New Roman','FontSize',12);

    figureHandle = gcf;
    %# make all text in the figure to size 14 and bold
    set(findall(figureHandle,'type','text'),'FontName','Times New Roman','FontSize',12)
    
    colorbar('FontName','Times New Roman','FontSize',12);
    
elseif position_and_size==2
    figure_obj.Position=[1+10 50 size(3:4)*0.4];
elseif position_and_size==3
    figure_obj.Position=[size(3)*0.4+10 50 size(3:4)*0.4];
elseif position_and_size==4
    figure_obj.Position=[1+10 size(3)*0.25 size(3:4)*0.4];
elseif position_and_size==5
    figure_obj.Position=[size(3)*0.4*(position_and_size-2) 2 size(3:4)*0.4];
elseif position_and_size==6
    figure_obj.Position=[size(3)*0.4*(position_and_size-2) size(4)*0.4*(position_and_size-2) size(3:4)*0.4];
end

output_args=1;
end


    function [ output_args ] = butterworth_filter(filter_kind,plot_switch,n_of_butt, input_args , frs, cutoff, high_cutoff)
%BUTTERWORTH_FILTER Summary of this function goes here
% 1.set the filter kind, 0 is low pass, other is band pass,
% 2.decide the n of butt,4 or 3 is ok
% 3.frs is need to calcute the time
% 4.if you are the low pass, only cutoff is need, for band pass, cutoff and high_cutoff are both needed

% NOTICE only can filter one row data

%   Detailed explanation goes here

s=input_args;
Length_of_data=length(s);
number_serial_of_data=0:Length_of_data-1;
time_serial_of_data=number_serial_of_data/frs;

    if filter_kind == 0% 0 is ditong
       
        sfft=fft(s);
        

        if 0
        Wp=cutoff/frs;Ws=high_cutoff/frs;                %cutoff 1Hz,stop_band cut off 2Hz
        %estimate the Butterworth N and 3dB cutoff frequence Wn
        [n,Wn]=buttord(Wp,Ws,1,50);     %Stop-band attenuation is greater than 50db and pass-band ripple is less than 1db.
        else
        n=n_of_butt;
        Wn=cutoff/(frs/2);%directly give cut off
        end
        %Butterworth
        [a,b]=butter(n,Wn);
        [h,f]=freqz(a,b,'whole',frs);      
        
        f=(0:length(f)-1*frs/length(f));   
        
%         sF=filter(a,b,s);                   %filter
        sF=filtfilt(a,b,s);                   %filter which has zero phase move
        
        
        SF=fft(sF);
        
        if plot_switch==1
             figure
            set(gcf,'outerposition',get(0,'screensize'));
            subplot(6,2,[2 4 6]);plot(time_serial_of_data,s);
            title('input signal');xlabel('t/s');ylabel('amplitude');
            subplot(6,2,[1 3]);
            plot((1:length(sfft)/2)*frs/length(sfft),2*abs(sfft(1:length(sfft)/2))/length(sfft));
            title('signal spectrum');xlabel('Hz');ylabel('amplitude');
            subplot(6,2,[5 7])
            plot(f(1:length(f)/2),abs(h(1:length(f)/2)));       %Draw amplitude-frequency response diagram
            title('Butterworth low pass filter');xlabel('Frequency/Hz');ylabel('Amplitude');
            grid;
            subplot(6,2,[8 10 12])
            plot(time_serial_of_data,sF);                         
            title('out put signal');xlabel('t/s');ylabel('Amplitude');
            subplot(6,2,[9 11])
            plot((1:length(SF)/2)*frs/length(SF),2*abs(SF(1:length(SF)/2))/length(SF));
            title('Low-pass filtered spectrum');xlabel('Frequency/Hz');ylabel('Amplitude');
        else
            
        end
    
        
    else    %else is daitong
        
        sfft=fft(s);
        

        n=n_of_butt;
        Wn=[cutoff/(frs/2) high_cutoff/(frs/2)];   %low and high cut off 

        %Butterworth
        [a,b]=butter(n,Wn);
        [h,f]=freqz(a,b,'whole',frs);       
        f=(0:length(f)-1*frs/length(f));   
        
%         sF=filter(a,b,s);                   %filter
        sF=filtfilt(a,b,s);                   %filter which has zero phase move
        
        
        SF=fft(sF);
        
        if plot_switch==1
            figure
            set(gcf,'outerposition',get(0,'screensize'));
            subplot(6,2,[2 4 6]);plot(time_serial_of_data,s);
            title('input signal');xlabel('t/s');ylabel('amplitude');
            subplot(6,2,[1 3]);
            plot((1:length(sfft)/2)*frs/length(sfft),2*abs(sfft(1:length(sfft)/2))/length(sfft));
            title('signal spectrum');xlabel('Hz');ylabel('amplitude');
            subplot(6,2,[5 7])
            plot(f(1:length(f)/2),abs(h(1:length(f)/2)));       %Draw amplitude-frequency response diagram
            title('Butterworth band pass filter');xlabel('Frequency/Hz');ylabel('Amplitude');
            grid;
            subplot(6,2,[8 10 12])
            plot(time_serial_of_data,sF);                         
            title('out put signal');xlabel('t/s');ylabel('Amplitude');
            subplot(6,2,[9 11])
            plot((1:length(SF)/2)*frs/length(SF),2*abs(SF(1:length(SF)/2))/length(SF));
            title('Band-pass filtered spectrum');xlabel('Frequency/Hz');ylabel('Amplitude');
        else
            
        end
        
    end

    

    output_args=sF;



    end
 
    
    function [ a] = plot_vel( Data_pose_vel,vel_samples,vel_size )
%UNTITLED 锟剿达拷锟斤拷示锟叫关此猴拷锟斤拷锟秸?
%   锟剿达拷锟斤拷示锟斤拷细说锟斤拷
figure
subplot(121)
%2d
    h_data = plot(Data_pose_vel(1,:),Data_pose_vel(3,:),'r.','markersize',10); hold on;
    h_att = [];
    att=[Data_pose_vel(1,end);Data_pose_vel(3,end)];
    h_att = scatter(att(1),att(2),150,[0 0 0],'d','Linewidth',2); hold on;
    
    % Plot Velocities of Reference Trajectories
    vel_points = Data_pose_vel([1 3 4 6],1:vel_samples:end);
    U = zeros(size(vel_points,2),1);
    V = zeros(size(vel_points,2),1);
    for i = 1:size(vel_points, 2)
        dir_    = vel_points(3:end,i);%/norm(vel_points(3:end,i))
        U(i,1)   = dir_(1);
        V(i,1)   = dir_(2);
    end
    h_vel = quiver(vel_points(1,:)',vel_points(2,:)', U, V, vel_size, 'Color', 'k', 'LineWidth',2); hold on;
    grid on;
    box on;
    
subplot(122)
    h_data = plot3(Data_pose_vel(1,:),Data_pose_vel(2,:),Data_pose_vel(3,:),'r.','markersize',10); hold on;
    h_att = [];
    att=[Data_pose_vel(1,end),Data_pose_vel(2,end),Data_pose_vel(3,end)];
    h_att = scatter3(att(1),att(2),att(3),150,[0 0 0],'d','Linewidth',2); hold on;
    
    % Plot Velocities of Reference Trajectories
    vel_points = Data_pose_vel(:,1:vel_samples:end);
    U = zeros(size(vel_points,2),1);
    V = zeros(size(vel_points,2),1);
    W = zeros(size(vel_points,2),1);
    for i = 1:size(vel_points, 2)
        dir_    = vel_points(4:end,i);%/norm(vel_points(4:end,i))
        U(i,1)   = dir_(1);
        V(i,1)   = dir_(2);
         W(i,1)   = dir_(3);
    end
    h_vel = quiver3(vel_points(1,:)',vel_points(2,:)',vel_points(3,:)', U, V, W,vel_size, 'Color', 'k', 'LineWidth',2); hold on;
    grid on;axis equal;
    box on;
a=1;
end


