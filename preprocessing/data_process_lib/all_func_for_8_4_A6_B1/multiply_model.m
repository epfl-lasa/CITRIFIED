function [out]=multiply_model(vel_input,vel_points_input)
   out=(vel_input/norm(vel_input))*vel_points_input*0.7;%
end
