%% EN.530.646 RDKDC - Final Project
% _*Group 7*_


%% IKerror(gdesired, g0)
% accepts g_desired and g_actual
% returns the positional error in [m] and rotational error in [deg]

function [err_pos, err_rot] = IKerror(gdesired, gactual)

t_desired = gdesired(1:3,4);
R_desired = gdesired(1:3,1:3); 
t_loc = gactual(1:3,4);
R_loc = gactual(1:3,1:3); 
err_pos = 100 * norm(t_loc - t_desired);
err_rot = rad2deg(sqrt(trace((R_loc - R_desired)*(R_loc - R_desired).')));

end

