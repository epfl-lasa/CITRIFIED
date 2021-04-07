
function [t, data] = plotJSONTrialData(filename,TYPE,SIGNAL)

file = fopen(filename);

phase = {};
esn = {};
force_raw = [];
force_filt = [];
vel_raw = [];
vel_filt = [];
pos = [];
t = [];


while ~feof(file)
    
    line = fgetl(file);
    message = jsondecode(line);
    
    
    if isfield(message, 'time')
        t = [t, message.time];
    end
    
    if isfield(message, 'esn')
        esn{end+1} = message.esn;
    end
    
    if isfield(message, 'control')
        phase{end+1} = message.control.phase;
    end
    
    
    switch lower(TYPE)
        
        case 'velocity'
            
            switch lower(SIGNAL)
                
                case 'raw'
                    
                    if isfield(message, 'raw')
                        vel_raw(end+1,:) = (message.raw.bodies{1,1}.twist.linear)';
                    end
                    
                case 'filtered'
                    
                    if isfield(message, 'filtered')
                        vel_filt(end+1,:) = (message.filtered.bodies{1,1}.twist.linear)';
                    end
            end
            
        case 'force'
            
            switch lower(SIGNAL)
                
                case 'raw'
                    
                    if isfield(message, 'raw')
                        force_raw(end+1,:) = (message.raw.bodies{4,1}.wrench.force)';
                    end
                    
                case 'filtered'
                    
                    if isfield(message, 'filtered')
                        force_filt(end+1,:) = (message.filtered.bodies{3,1}.wrench.force)';
                    end
            end
            
            
        case 'position'
            
            switch lower(SIGNAL)
                
                case {'raw','filtered'}
                    
                    
                    if isfield(message, 'raw')
                        pos(end+1,:) = (message.raw.bodies{1,1}.pose.position)';
                    end
            end
    end
end

fclose(file);


%% Plots

% Velocity

switch lower(TYPE)
    
    case 'velocity'
        
        switch lower(SIGNAL)
            
            case 'raw'
                
                data = vel_raw;
                
                subplot(3,1,1)
                plot(t,vel_raw(:,1))
                title('$V_x$','interpreter','latex')
                xlabel('s','interpreter','latex')
                ylabel('m/s','interpreter','latex')
                
                subplot(3,1,2)
                plot(t,vel_raw(:,2))
                title('$V_y$','interpreter','latex')
                xlabel('s','interpreter','latex')
                ylabel('m/s','interpreter','latex')
                
                subplot(3,1,3)
                plot(t,vel_raw(:,3))
                title('$V_z$','interpreter','latex')
                xlabel('s','interpreter','latex')
                ylabel('m/s','interpreter','latex')
                
                sgtitle('Velocity of ee in robot frame (raw signal)','interpreter','latex')
                
            case 'filtered'
                
                data = vel_filt;
                
                subplot(3,1,1)
                plot(t,vel_filt(:,1))
                title('$V_x$','interpreter','latex')
                xlabel('s','interpreter','latex')
                ylabel('m/s','interpreter','latex')
                
                subplot(3,1,2)
                plot(t,vel_filt(:,2))
                title('$V_y$','interpreter','latex')
                xlabel('s','interpreter','latex')
                ylabel('m/s','interpreter','latex')
                
                subplot(3,1,3)
                plot(t,vel_filt(:,3))
                title('$V_z$','interpreter','latex')
                xlabel('s','interpreter','latex')
                ylabel('m/s','interpreter','latex')
                
                sgtitle('Velocity of ee in robot frame (filtered signal)','interpreter','latex')
                
        end
        
        % Force
        
    case 'force'
        
        switch lower(SIGNAL)
            
            case 'raw'
                
                data = force_raw;
                
                subplot(3,1,1)
                plot(t,force_raw(:,1))
                title('$F_x$','interpreter','latex')
                xlabel('s','interpreter','latex')
                ylabel('N','interpreter','latex')
                
                subplot(3,1,2)
                plot(t,force_raw(:,2))
                title('$F_y$','interpreter','latex')
                xlabel('s','interpreter','latex')
                ylabel('N','interpreter','latex')
                
                subplot(3,1,3)
                plot(t,force_raw(:,3))
                title('$F_z$','interpreter','latex')
                xlabel('s','interpreter','latex')
                ylabel('N','interpreter','latex')
                
                sgtitle('Force (raw signal)','interpreter','latex')
                
            case 'filtered'
                
                data = force_filt;
                
                subplot(3,1,1)
                plot(t,force_filt(:,1))
                title('$F_x$','interpreter','latex')
                xlabel('s','interpreter','latex')
                ylabel('N','interpreter','latex')
                
                subplot(3,1,2)
                plot(t,force_filt(:,2))
                title('$F_y$','interpreter','latex')
                xlabel('s','interpreter','latex')
                ylabel('N','interpreter','latex')
                
                subplot(3,1,3)
                plot(t,force_filt(:,3))
                title('$F_z$','interpreter','latex')
                xlabel('s','interpreter','latex')
                ylabel('N','interpreter','latex')
                
                sgtitle('Force (filtered signal)','interpreter','latex')
                
        end
        
        % Position
        
    case 'position'
        
        switch lower(SIGNAL)
            
            case {'raw','filtered'}
                
                data = pos;
                
                figure
                plot3(pos(:,1),pos(:,2),pos(:,3))
                axis equal
                axis vis3d
                title('Position','interpreter','latex')
                xlabel('x','interpreter','latex')
                ylabel('y','interpreter','latex')
                zlabel('z','interpreter','latex')
                
        end
end

end