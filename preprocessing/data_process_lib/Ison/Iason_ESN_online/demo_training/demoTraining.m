clear all
clc
close all
%%

% set subject's ID
sbj=1;

% training method (1 or 2)

tMeth=2;


% setting the target for reading EMG data

read_target=['C:\Users\Iason\Documents\Visual Studio 2013\Projects\demo_project\demo_project\data\sbj' num2str(sbj) '\trial'];                        

 % setting the target for writing the trained ESNs

save_target=['C:\Users\Iason\Documents\Visual Studio 2013\Projects\demo_project\demo_project\data\networks\sbj'  num2str(sbj) '\ESN'];                           

targetName='tw';

nb_trials=20;                                           % setting the number of trials per class
nb_class=5;                                             % setting the number of classes
fs=1000;                                                % setting sampling rate
tw=0.15;                                                % setting the time window length
delay=0.1;                                              % setting the overlap

nUn=70;                                                % nb of units

maxTW=15;                                               % maximun number of time windows




%% collect data



data2=struct([]);
labels2=struct([]);

tmWs=struct([]);

allLabels=struct([]);

counter=1;

if (tMeth==1)

for timeWindow=0:maxTW  
    
    data=struct([]);
    labels=struct([]);
    
    for i=1:nb_trials*nb_class
    
        load([read_target num2str(i) '\timeWindows\' targetName num2str(timeWindow) '.mat'])
    
        data{i}=twM;
        data2{counter}=twM;
        
        labels{i}=zeros(length(tmW),nb_class);
        labels{i}(:,floor((i-1)/nb_trials)+1)=1;
        
        labels2{counter}=zeros(length(tmW),nb_class);
        labels2{counter}(:,floor((i-1)/nb_trials)+1)=1;
        
        counter=counter+1;
        
    end
    tmWs{timeWindow+1}=data;
    allLabels{timeWindow+1}=labels;
        
end
end

%%

if (tMeth==2)

for timeWindow=0:maxTW  
    
    data=struct([]);
    labels=struct([]);
    
    for i=1:nb_trials*nb_class
    i
        load([read_target num2str(i) '\timeWindows\' targetName num2str(timeWindow) '.mat'])
    
        data{i}=twM;
        data2{counter}=twM;
        
        labels{i}=zeros(length(twM),nb_class);
        if(rem(i,nb_class)==0)
            labels{i}(:,nb_class)=1;
        else
            labels{i}(:,rem(i,nb_class))=1;
        end
      
        labels2{counter}=zeros(length(twM),nb_class);
        
        
        if(rem(i,nb_class)==0)
            labels2{counter}(:,nb_class)=1;
        else
            labels2{counter}(:,rem(i,nb_class))=1;
        end
        
        counter=counter+1;
        
    end
    tmWs{timeWindow+1}=data;
    allLabels{timeWindow+1}=labels;
        
end
end









%% first method
%% train the system

esn_nets=struct([]);
sc=struct([]);

for i=1:maxTW
    
    [sc{i},esn_nets{i}]=ESNtraining(tmWs{i},allLabels{i},nUn,i,30,save_target);

end


data2test=struct([]);
labels2test=struct([]);

tmWstest=struct([]);

allLabelstest=struct([]);


%%
if (tMeth==2)
    

conter=1;

for timeWindow=0:maxTW-2 
    
    data=struct([]);
    labels=struct([]);
    
    %for i=nb_trials*nb_class+1:150
    for i=nb_trials*nb_class
    
        load([read_target num2str(i) '\timeWindows\' targetName num2str(timeWindow) '.mat'])
    
        data{i-nb_trials*nb_class}=twM;
        data2test{counter}=twM;
        
        labels{i-nb_trials*nb_class}=zeros(length(twM),nb_class);
        
        if(rem(i,nb_class)==0)
            labels{i-nb_trials*nb_class}(:,nb_class)=1;
        else
            labels{i-nb_trials*nb_class}(:,rem(i,nb_class))=1;
        end
      
        if(rem(i,nb_class)==0)
            labels2test{counter}(:,nb_class)=1;
        else
            labels2test{counter}(:,rem(i,nb_class))=1;
        end
        
        counter=counter+1;
        
    end
    tmWstest{timeWindow+1}=data;
    allLabelstest{timeWindow+1}=labels;
        
end

for i=1:maxTW-2
    [score_test]=testESN2(tmWstest{i},allLabelstest{i},maxTW,nUn,esn_nets{i},30);
end
end





%% second method
%% train the system

    
[sc2,esn_nets2]=ESNtraining(data2,labels2,10,maxTW+2,30,save_target);

%%
[score_train]=testESN(tmWs,allLabels,maxTW,nUn,esn_nets2,30);

%%
if(tMeth==2)
        
    [score_test]=testESN(tmWstest,allLabelstest,maxTW,nUn,esn_nets2,30);
    
    
end


