function a=writeESNtofile(ESN, file, nfp)

fileID=fopen(file,'w');

fprintf(fileID, '%d ',nfp);

fprintf(fileID, '%d ',[ESN.nInternalUnits,ESN.nInputUnits,ESN.nOutputUnits]);

%%%
t=full(ESN.internalWeights_UnitSR);

for i=1:size(t,1)
    
    fprintf(fileID, '%f \n',t(1,:));
end

%%%
fprintf(fileID,'%d ',ESN.nTotalUnits);

%%%
t=full(ESN.inputWeights);
for i=1:size(t,1)
    fprintf(fileID, '%f ',t(i,:));
end

%%%
t=full(ESN.outputWeights);
for i=1:size(t,1)
    fprintf(fileID, '%f ',t(i,:));
end

%%%
t=full(ESN.feedbackWeights);
for i=1:size(t,1)
    fprintf(fileID, '%f ',t(i,:));
end

%%%
t=full(ESN.inputScaling);
for i=1:size(t,1)
    fprintf(fileID, '%f ',t(i,:));
end

%%%
t=full(ESN.inputShift);
for i=1:size(t,1)
    fprintf(fileID, '%f ',t(i,:));
end

%%%
t=full(ESN.teacherScaling);
for i=1:size(t,1)
    fprintf(fileID, '%f ',t(i,:));
end


%%%
t=full(ESN.teacherShift);
for i=1:size(t,1)
    fprintf(fileID, '%f ',t(i,:));
end

%%%
t=full(ESN.feedbackScaling);
for i=1:size(t,1)
    fprintf(fileID, '%f ',t(i,:));
end

%%%
t=full(ESN.timeConstants);
for i=1:size(t,1)
    fprintf(fileID, '%f ',t(i,:));
end

%%%
fprintf(fileID,'%d ',ESN.leakage);

%%%
t=full(ESN.internalWeights);
for i=1:size(t,1)
    fprintf(fileID, '%f ',t(i,:));
end





fclose(fileID);
a=1;
end