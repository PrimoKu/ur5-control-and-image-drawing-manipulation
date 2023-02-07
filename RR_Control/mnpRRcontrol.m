%% EN.530.646 RDKDC - Final Project
% _*Group 7*_


%% Function mnpRRcontrol
%
% Move and Place Task parent function for ur5RRcontrol
% 


function [error_start_pos, error_start_rot, error_target_pos, error_target_rot, t_duration] = mnpRRcontrol(g_start, g_start_above, g_target, g_target_above, g_im, ur5)
    
    % Gain
    k = 15.0;

    % Intermediate Home (joints and g)
    joints_imHome = [pi/6; -pi/2; pi/12; -pi/2; -pi/12; 0];
    if g_start(1,4) > 0
        joints_imHome(1) = joints_imHome(1)*-1;
        if g_start(2,4) > 0
            joints_imHome(3) = joints_imHome(3)*-1;
            joints_imHome(5) = joints_imHome(5)*-1;
        end
    else
        if g_start(2,4) < 0
            joints_imHome(3) = joints_imHome(3)*-1;
            joints_imHome(5) = joints_imHome(5)*-1;
        end
    end
    g_imHome = ur5FwdKin(joints_imHome);

    %% ur5.home -> imHome
    fprintf('\n-> UR5 started moving to intermediate home configuration.\n');
    ur5.move_joints(joints_imHome,5);   pause(6);
    fprintf('<- UR5 moved to intermediate home configuration.\n');

    %% imHome -> start_above
    tic;
    fprintf('\n-> UR5 started moving to g_start_above.\n');
    ur5RRcontrol(g_start_above, k, ur5);
    fprintf('<- UR5 moved to g_start_above.\n');
    
    %% start_above -> start
    fprintf('\n-> UR5 started moving to g_start.\n');
    [error_start_pos, error_start_rot] = ur5RRcontrol(g_start, k, ur5);
    fprintf('<- UR5 moved to g_start.\n');

    %% start -> start_above
    fprintf('\n-> UR5 started moving to g_start_above.\n');
    ur5RRcontrol(g_start_above, k, ur5);
    fprintf('<- UR5 moved to g_start_above.\n');

    %% start_above -> intermediate configuration
    fprintf('\n-> UR5 started moving to g_im.\n');
    ur5RRcontrol(g_im, k, ur5);
    fprintf('<- UR5 moved to g_im.\n');
    
    %% intermidiate configuratin -> target_above
    fprintf('\n-> UR5 started moving to g_target_above.\n');
    ur5RRcontrol(g_target_above, k, ur5);
    fprintf('<- UR5 moved to g_target_above.\n');

    %% target_above -> target
    fprintf('\n-> UR5 started moving to g_target.\n');
    [error_target_pos, error_target_rot] = ur5RRcontrol(g_target, k, ur5);
    fprintf('<- UR5 moved to g_target.\n');

    %% target -> target_above
    fprintf('\n-> UR5 started moving to g_target_above.\n');
    ur5RRcontrol(g_target_above, k, ur5);
    fprintf('<- UR5 moved to g_target_above.\n');
    t_duration = toc;

    %% target_above -> imHome
    fprintf('\n-> UR5 started moving to intermediate home configuration.\n');
    ur5.move_joints(joints_imHome, 5);
    fprintf('<- UR5 moved to intermediate home configuration.\n');

end