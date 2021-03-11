function out = export2GMMGMR_Cpp_lib_PvI_add_imp_last_first_increase_imp(name,x_first,x_last,Mu,Sigma,imp,imp_first, imp_last)
% dlmwrite(name, size(Mu),'Delimiter',' ','precision','%i');
dlmwrite(name, x_first,'newline','pc','-append','Delimiter',' ','precision','%.6f');
dlmwrite(name, x_last,'newline','pc','-append','Delimiter',' ','precision','%.6f');
dlmwrite(name, imp_first*4,'newline','pc','-append','Delimiter',' ','precision','%.6f');
dlmwrite(name, imp_last*4,'newline','pc','-append','Delimiter',' ','precision','%.6f');
dlmwrite(name, Mu(1,:),'newline','pc','-append','Delimiter',' ','precision','%.6f');
dlmwrite(name, Mu(2,:),'newline','pc','-append','Delimiter',' ','precision','%.6f');
dlmwrite(name, Mu(3,:),'newline','pc','-append','Delimiter',' ','precision','%.6f');

dlmwrite(name, imp(1,:)*4,'newline','pc','-append','Delimiter',' ','precision','%.6f');
dlmwrite(name, imp(2,:)*4,'newline','pc','-append','Delimiter',' ','precision','%.6f');
dlmwrite(name, imp(3,:)*4,'newline','pc','-append','Delimiter',' ','precision','%.6f');
out = true;