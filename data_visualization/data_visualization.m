
function [phase,esn,force_raw,force_filt,vel_raw,vel_filt,pos,t]= data_visualization(filename,TYPE,SIGNAL) 

file = fopen(filename);

phase = {};

esn = {};

force_raw = [];
force_filt = [];

vel_raw = [];
vel_filt = [];

pos = [];

t = [];

esn_ind = 1;
phase_ind = 1;
force_r_ind = 1;
force_f_ind = 1;
vel_r_ind = 1;
vel_f_ind = 1;
pos_ind = 1;

message1 = jsondecode(fgetl(file));

while ~feof(file)
    
    line = fgetl(file);
    message = jsondecode(line);
    
    t = [t; message.time];
 
    
    if isfield(message, 'esn')
        esn{esn_ind} = message.esn;
        esn_ind = esn_ind + 1;
    end
    
    if isfield(message, 'control')
        phase{phase_ind} = message.control.phase;
        phase_ind = phase_ind + 1;
    end

    
    
     if (TYPE == "velocity")
         
        if (SIGNAL == "raw") 
             
                 if isfield(message, 'raw')
                       vel_raw(vel_r_ind,:) = (message.raw.bodies{1,1}.twist.linear)';
                       vel_r_ind = vel_r_ind + 1;
                 end
                 
        elseif (SIGNAL == "filtered") 
            
                  if isfield(message, 'filtered')
                       vel_filt(vel_f_ind,:) = (message.filtered.bodies{1,1}.twist.linear)';
                       vel_f_ind = vel_f_ind + 1;
                  end
         
        end
        
     end
     
     
     if (TYPE == "force")
         
        if (SIGNAL == "raw") 
             
                  if isfield(message, 'raw')
                    force_raw(force_r_ind,:) = (message.raw.bodies{4,1}.wrench.force)';
                    force_r_ind = force_r_ind + 1;
                  end
                 
        elseif (SIGNAL == "filtered") 
            
                 if isfield(message, 'filtered')
                    force_filt(force_f_ind,:) = (message.filtered.bodies{3,1}.wrench.force)';
                    force_f_ind = force_f_ind + 1;
                 end
         
        end
        
     end
     
     if (TYPE == "position")
         
        if (SIGNAL == "raw" || SIGNAL == "filtered") 
             
                 if isfield(message, 'raw')
                       pos(pos_ind,:) = (message.raw.bodies{1,1}.pose.position)';
                       pos_ind = pos_ind + 1;
                 end
         
        end
        
     end
        

end


fclose(file);

t = t-t(1);


%% Plots

% Velocity 

 if (TYPE == "velocity")
         
        if (SIGNAL == "raw") 
             
                 subplot(3,1,1)
                 plot(t,vel_raw(1:end,1));
                 title('Vx - Raw')
                 xlabel('s','interpreter','latex')
                 ylabel('m/s','interpreter','latex')
                 
                 subplot(3,1,2)
                 plot(t,vel_raw(1:end,2));
                 title('Vy - Raw')
                 xlabel('s','interpreter','latex')
                 ylabel('m/s','interpreter','latex')
                 
                 subplot(3,1,3)
                 plot(t,vel_raw(1:end,3));
                 title('Vz - Raw')
                 xlabel('s','interpreter','latex')
                 ylabel('m/s','interpreter','latex')
                 
        elseif (SIGNAL == "filtered") 
            
                 subplot(3,1,1)
                 plot(t,vel_filt(1:end,1));
                 title('Vx - Filtered')
                 xlabel('s','interpreter','latex')
                 ylabel('m/s','interpreter','latex')
                 
                 subplot(3,1,2)
                 plot(t,vel_filt(1:end,2));
                 title('Vy - Filtered')
                 xlabel('s','interpreter','latex')
                 ylabel('m/s','interpreter','latex')
                 
                 subplot(3,1,3)
                 plot(t,vel_filt(1:end,3));
                 title('Vz - Filtered')
                 xlabel('s','interpreter','latex')
                 ylabel('m/s','interpreter','latex')
         
        end
        
 end
     
     
% Force 

     if (TYPE == "force")
         
        if (SIGNAL == "raw") 
             
                 subplot(3,1,1)
                 plot(t,force_raw(1:end,1));
                 title('Fx - Raw')
                 xlabel('s','interpreter','latex')
                 ylabel('N','interpreter','latex')

                 subplot(3,1,2)
                 plot(t,force_raw(1:end,2));
                 title('Fy - Raw')
                 xlabel('s','interpreter','latex')
                 ylabel('N','interpreter','latex')
                 
                 subplot(3,1,3)
                 plot(t,force_raw(1:end,3));
                 title('Fz - Raw')
                 xlabel('s','interpreter','latex')
                 ylabel('N','interpreter','latex')
                 
        elseif (SIGNAL == "filtered") 
            
                 subplot(3,1,1)
                 plot(t,force_filt(1:end,1));
                 title('Fx - Filtered')
                 xlabel('s','interpreter','latex')
                 ylabel('N','interpreter','latex')
                 
                 subplot(3,1,2)
                 plot(t,force_filt(1:end,2));
                 title('Fy - Filtered')
                 xlabel('s','interpreter','latex')
                 ylabel('N','interpreter','latex')
                 
                 subplot(3,1,3)
                 plot(t,force_filt(1:end,3));
                 title('Fz - Filtered')
                 xlabel('s','interpreter','latex')
                 ylabel('N','interpreter','latex')
         
        end
        
     end

% Position

if (TYPE == "position")
         
        if (SIGNAL == "raw" || SIGNAL == "filtered") 
             
             figure
             plot3(pos(:,1),pos(:,2),pos(:,3));
             title('Position')
             xlabel('x','interpreter','latex')
             ylabel('y','interpreter','latex')
             zlabel('z','interpreter','latex')
             
       
        end
        
end

end