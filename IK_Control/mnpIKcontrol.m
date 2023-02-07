%% EN.530.646 RDKDC - Final Project
% _*Group 7*_


%% Function mnpIKcontrol
%
% Move and Place Task parent function for ur5IKcontrol
% utilizing ur5InvKin function provided on canvas
% 


function [error_start_pos, error_start_rot, error_target_pos, error_target_rot, t_duration] = mnpIKcontrol(g_start, g_start_above, g_target, g_target_above, g_im, ur5)
    
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
    joints_current = ur5.get_current_joints();
    joints_InvKin = ur5InvKin(g_start_above);
    idx = minNormIdx(joints_InvKin, joints_current);

    % -- Safety Check: table collision check [Re-Route]
    % Fwd Kin of Joint 3
    gk_j3 = ur5FwdKin(joints_InvKin(:, idx),3);
    % First check: Z value from Joint 3 Fwd Kin && Joint 2 value
    if gk_j3(3,4) <= 0 || joints_InvKin(2, idx) >= pi/18 || joints_InvKin(2, idx) <= -17*pi/18
        joints_InvKin(2, idx) = -pi/2;
        fprintf('Table collision, re-route trajectory.\n');
    end
    % Second check: Z value from E-E Fwd Kin
    %   Joint 3 < 0 deg -> Joint 3 set to  pi/2
    %   Joint 3 > 0 deg -> Joint 3 set to -pi/2
    if g_start_above(3,4) <= 0
        joints_InvKin(3, idx) = -sign(joints_InvKin(3, idx))*pi/4;
        joints_InvKin(2, idx) = -pi/2;
        fprintf('Table collision, re-route trajectory.\n');
    end

    ur5.move_joints(joints_InvKin(:, idx), 5); pause(6);
    fprintf('<- UR5 moved to g_start_above.\n');
    
    %% start_above -> start
    fprintf('\n-> UR5 started moving to g_start.\n');
    joints_current = ur5.get_current_joints();
    joints_InvKin = ur5InvKin(g_start);
    idx = minNormIdx(joints_InvKin, joints_current);

    % -- Safety Check: table collision check [Re-Route]
    % Fwd Kin of Joint 3
    gk_j3 = ur5FwdKin(joints_InvKin(:, idx),3);
    % First check: Z value from Joint 3 Fwd Kin && Joint 2 value
    if gk_j3(3,4) <= 0 || joints_InvKin(2, idx) >= pi/18 || joints_InvKin(2, idx) <= -17*pi/18
        joints_InvKin(2, idx) = -pi/2;
        fprintf('Table collision, re-route trajectory.\n');
    end
    % Second check: Z value from E-E Fwd Kin
    %   Joint 3 < 0 deg -> Joint 3 set to  pi/2
    %   Joint 3 > 0 deg -> Joint 3 set to -pi/2
    if g_start(3,4) <= 0
        joints_InvKin(3, idx) = -sign(joints_InvKin(3, idx))*pi/4;
        joints_InvKin(2, idx) = -pi/2;
        fprintf('Table collision, re-route trajectory.\n');
    end

    ur5.move_joints(joints_InvKin(:, idx), 5); pause(6);
    fprintf('<- UR5 moved to g_start.\n');

    % Error
    joints_current = ur5.get_current_joints();
    g_current = ur5FwdKin(joints_current);
    [error_start_pos, error_start_rot] = IKerror(g_start, g_current);

    %% start -> start_above
    fprintf('\n-> UR5 started moving to g_start_above.\n');
    joints_current = ur5.get_current_joints();
    joints_InvKin = ur5InvKin(g_start_above);
    idx = minNormIdx(joints_InvKin, joints_current);
    
    % -- Safety Check: table collision check [Re-Route]
    % Fwd Kin of Joint 3
    gk_j3 = ur5FwdKin(joints_InvKin(:, idx),3);
    % First check: Z value from Joint 3 Fwd Kin && Joint 2 value
    if gk_j3(3,4) <= 0 || joints_InvKin(2, idx) >= pi/18 || joints_InvKin(2, idx) <= -17*pi/18
        joints_InvKin(2, idx) = -pi/2;
        fprintf('Table collision, re-route trajectory.\n');
    end
    % Second check: Z value from E-E Fwd Kin
    %   Joint 3 < 0 deg -> Joint 3 set to  pi/2
    %   Joint 3 > 0 deg -> Joint 3 set to -pi/2
    if g_start_above(3,4) <= 0
        joints_InvKin(3, idx) = -sign(joints_InvKin(3, idx))*pi/4;
        joints_InvKin(2, idx) = -pi/2;
        fprintf('Table collision, re-route trajectory.\n');
    end
    
    ur5.move_joints(joints_InvKin(:, idx), 5); pause(6);
    fprintf('<- UR5 moved to g_start_above.\n');
    
    %% intermidiate configuratin -> target_above
    fprintf('\n-> UR5 started moving to g_target_above.\n');
    joints_current = ur5.get_current_joints();
    joints_InvKin = ur5InvKin(g_target_above);
    idx = minNormIdx(joints_InvKin, joints_current);

    % -- Safety Check: table collision check [Re-Route]
    % Fwd Kin of Joint 3
    gk_j3 = ur5FwdKin(joints_InvKin(:, idx),3);
    % First check: Z value from Joint 3 Fwd Kin && Joint 2 value
    if gk_j3(3,4) <= 0 || joints_InvKin(2, idx) >= pi/18 || joints_InvKin(2, idx) <= -17*pi/18
        joints_InvKin(2, idx) = -pi/2;
        fprintf('Table collision, re-route trajectory.\n');
    end
    % Second check: Z value from E-E Fwd Kin
    %   Joint 3 < 0 deg -> Joint 3 set to  pi/2
    %   Joint 3 > 0 deg -> Joint 3 set to -pi/2
    if g_target_above(3,4) <= 0
        joints_InvKin(3, idx) = -sign(joints_InvKin(3, idx))*pi/4;
        joints_InvKin(2, idx) = -pi/2;
        fprintf('Table collision, re-route trajectory.\n');
    end

    ur5.move_joints(joints_InvKin(:, idx), 5); pause(6);
    fprintf('<- UR5 moved to g_target_above.\n');

    %% target_above -> target
    fprintf('\n-> UR5 started moving to g_target.\n');
    joints_current = ur5.get_current_joints();
    joints_InvKin = ur5InvKin(g_target);
    idx = minNormIdx(joints_InvKin, joints_current);

    % -- Safety Check: table collision check [Re-Route]
    % Fwd Kin of Joint 3
    gk_j3 = ur5FwdKin(joints_InvKin(:, idx),3);
    % First check: Z value from Joint 3 Fwd Kin && Joint 2 value
    if gk_j3(3,4) <= 0 || joints_InvKin(2, idx) >= pi/18 || joints_InvKin(2, idx) <= -17*pi/18
        joints_InvKin(2, idx) = -pi/2;
        fprintf('Table collision, re-route trajectory.\n');
    end
    % Second check: Z value from E-E Fwd Kin
    %   Joint 3 < 0 deg -> Joint 3 set to  pi/2
    %   Joint 3 > 0 deg -> Joint 3 set to -pi/2
    if g_target(3,4) <= 0
        joints_InvKin(3, idx) = -sign(joints_InvKin(3, idx))*pi/4;
        joints_InvKin(2, idx) = -pi/2;
        fprintf('Table collision, re-route trajectory.\n');
    end

    ur5.move_joints(joints_InvKin(:, idx), 5); pause(6);
    fprintf('<- UR5 moved to g_target.\n');

    % Error
    joints_current = ur5.get_current_joints();
    g_current = ur5FwdKin(joints_current);
    [error_target_pos, error_target_rot] = IKerror(g_target, g_current);

    %% target -> target_above
    fprintf('\n-> UR5 started moving to g_target_above.\n');
    joints_current = ur5.get_current_joints();
    joints_InvKin = ur5InvKin(g_target_above);
    idx = minNormIdx(joints_InvKin, joints_current);
    
    % -- Safety Check: table collision check [Re-Route]
    % Fwd Kin of Joint 3
    gk_j3 = ur5FwdKin(joints_InvKin(:, idx),3);
    % First check: Z value from Joint 3 Fwd Kin && Joint 2 value
    if gk_j3(3,4) <= 0 || joints_InvKin(2, idx) >= pi/18 || joints_InvKin(2, idx) <= -17*pi/18
        joints_InvKin(2, idx) = -pi/2;
        fprintf('Table collision, re-route trajectory.\n');
    end
    % Second check: Z value from E-E Fwd Kin
    %   Joint 3 < 0 deg -> Joint 3 set to  pi/2
    %   Joint 3 > 0 deg -> Joint 3 set to -pi/2
    if g_target_above(3,4) <= 0
        joints_InvKin(3, idx) = -sign(joints_InvKin(3, idx))*pi/4;
        joints_InvKin(2, idx) = -pi/2;
        fprintf('Table collision, re-route trajectory.\n');
    end
    
    ur5.move_joints(joints_InvKin(:, idx), 5); pause(6);
    fprintf('<- UR5 moved to g_target_above.\n');
    t_duration = toc;

    %% target_above -> imHome
    fprintf('\n-> UR5 started moving to intermediate home configuration.\n');
    ur5.move_joints(joints_imHome, 5); pause(6);
    fprintf('<- UR5 moved to intermediate home configuration.\n');

end
