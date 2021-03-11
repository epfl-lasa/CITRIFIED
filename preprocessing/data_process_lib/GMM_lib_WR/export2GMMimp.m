function out = export2GMMposition(name,imp,imp_Sigma,Priors)
dlmwrite(name, size(imp),'Delimiter',' ','precision','%i'); %size of Mu(6*K(GM))
dlmwrite(name, Priors,'newline','pc','-append','Delimiter',' ','precision','%.6f');
%dlmwrite(name, reshape(Mu/1000,1,[]),'newline','pc','-append','Delimiter',' ','precision','%.6f');
dlmwrite(name, reshape(imp,1,[]),'newline','pc','-append','Delimiter',' ','precision','%.6f');
for i=1:size(imp,2)
    %dlmwrite(name, reshape(Sigma(:,:,i)/1000^2,1,[]),'newline','pc','-append','Delimiter',' ','precision','%.6f');
    dlmwrite(name, reshape(imp_Sigma(:,:,i)',1,[]),'newline','pc','-append','Delimiter',' ','precision','%.6f');
end
out = true;
