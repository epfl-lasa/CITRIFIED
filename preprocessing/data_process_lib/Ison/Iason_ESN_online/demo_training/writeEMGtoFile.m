function op=writeEMGtoFile(file,EMG)

[a,d]=size(EMG);

fileID=fopen(file,'w');

for i=1:a
    for j=1:d
        fprintf(fileID, '%f \n',EMG(i,j));
    end
end
fclose(fileID);


op=1;


end