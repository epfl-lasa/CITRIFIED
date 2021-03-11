function out = export2GMMGMR_Cpp_lib(name,x_first,x_last,Mu,Sigma)
% dlmwrite(name, size(Mu),'Delimiter',' ','precision','%i');
dlmwrite(name, x_first,'newline','pc','-append','Delimiter',' ','precision','%.6f');
dlmwrite(name, x_last,'newline','pc','-append','Delimiter',' ','precision','%.6f');
dlmwrite(name, Mu(1,:),'newline','pc','-append','Delimiter',' ','precision','%.6f');
dlmwrite(name, Mu(2,:),'newline','pc','-append','Delimiter',' ','precision','%.6f');
dlmwrite(name, Mu(3,:),'newline','pc','-append','Delimiter',' ','precision','%.6f');
out = true;