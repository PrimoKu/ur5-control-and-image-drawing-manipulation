%% EN.530.646 RDKDC - Final Project
% _*Group 7*_


%% Document Setup

% -- Clear command window, workspace and close figures
clc; clear; close all;

% -- Add Path
addpath('UR5_basics/');
addpath('basic_functions/');
addpath('IK_Control/');
addpath('RR_Control/');
addpath('TJ_Control/');
addpath('Extra_Credit/');

% -- Set output display format (for testing)
% format default
% format shortE % Scientific


%% Initialize ur5 object

ur5 = ur5_interface();


%% UR5 Physical Configuration

% Workspace
ur5_WS_min = 0.20;      
ur5_WS_max = 0.85; 

% Offsets
above_offset = 0.15;
midxy_offset = 0.25;
midz_offset = 1 * above_offset;


%% Main Script
while true
    
    % -- Teach UR5
   
    % Move and Place Task
    loop_Project =  input('\n---------- Terminate ur5_project? [Y/N] ? ----------\n','s');
    if strcmp('Y', loop_Project) ==  1 || strcmp('y', loop_Project) ==  1
        break;
    end
    % Extra Credit (Draw JHU Logo)
    loop_EC =  input('\n---------- Demonstration for extra credit? [Y/N] ? ----------\n','s');
    if strcmp('Y', loop_EC) ==  1 || strcmp('y', loop_EC) ==  1
        % draw JHU Logo!
        joints_imgOrigin = ur5.home + [5*pi/12; pi/6; 2*pi/3; -pi/3; -pi/2; 0];
        g_imgOrigin = ur5FwdKin(joints_imgOrigin);
        Msg = drawJHULogo(g_imgOrigin, ur5);
        break;
    end
    % start location
    g_start = input(['\n---------- Input desired start location g_start ----------\n' ...
                     '[R] to teach and record the current joint\n'...
                     '[T] to use test start location provided\n'...
                     '[P] to use previous g_start (if defined)\n'...
                     '[-] enter g matrix directly\n'],'s');
    if strcmp('R', g_start) == 1 || strcmp('r', g_start) ==  1
        g_start = ur5FwdKin(ur5.get_current_joints());
    elseif strcmp('T', g_start) == 1 || strcmp('t', g_start) ==  1
        g_start = [[ 0 -1  0  0.30];
                   [-1  0  0 -0.40]; 
                   [ 0  0 -1  0.22]; 
                   [ 0  0  0  1   ]];
%         g_start = [[ 0.4838  0.8752  0.0011 -0.4557];
%                    [ 0.8744 -0.4833 -0.0437  0.3736]; 
%                    [-0.0377  0.0221 -0.9990  0.0997]; 
%                    [ 0       0       0       1     ]];
    elseif strcmp('P', g_start) == 1 || strcmp('p', g_start) ==  1
        g_start = g_start_prev;
    else
        g_start = str2num(g_start);
    end
    g_start_prev = g_start;
    % end location
    g_target = input(['\n---------- Input desired target location g_target ----------\n' ...
                      '[R] to teach and record the current joint\n'...
                      '[T] to use test target location provided\n'...
                      '[P] to use previous g_target (if defined)\n'...
                      '[-] enter g matrix directly\n'],'s');
    if strcmp('R', g_target) == 1 || strcmp('r', g_target) ==  1
        g_target = ur5FwdKin(ur5.get_current_joints());
    elseif strcmp('T', g_target) == 1 || strcmp('t', g_target) ==  1
        g_target = [[ 0 -1  0 -0.30]; 
                    [-1  0  0  0.39]; 
                    [ 0  0 -1  0.22]; 
                    [ 0  0  0  1   ]];
%         g_target = [[ 0.7464 -0.6641 -0.0426  0.5089];
%                     [-0.6645 -0.7472  0.0058  0.4116]; 
%                     [-0.0357  0.0240 -0.9991  0.1303]; 
%                     [ 0       0       0       1     ]];
    elseif strcmp('P', g_target) == 1 || strcmp('p', g_target) ==  1
        g_target = g_target_prev;
    else 
        g_target = str2num(g_target);
    end
    g_target_prev = g_target;


    % -- UR5 Configurations
    g_above = [zeros(3) [0;0;above_offset]; zeros(1,4)];
    g_start_above = g_start + g_above;
    g_target_above = g_target + g_above;
    

    % -- place frames for desired configurations
    % frame start
    frame_start = tf_frame('base_link', 'start', g_start); pause(1);
    % frame start above
    frame_start_above = tf_frame('base_link', 'start_above', g_start_above); pause(1);
    % frame target
    frame_target = tf_frame('base_link', 'target', g_target); pause(1);
    % frame target above
    frame_target_above = tf_frame('base_link', 'target_above', g_target_above); pause(1);
    % complete msg
    fprintf('\n---------- Frames initiation completed ----------\n');


    % -- Safety Check
    % Workspace check
    if norm(       g_start(1:3,4)) > ur5_WS_max ||        norm(g_start(1:2,4)) < ur5_WS_min ||...
       norm( g_start_above(1:3,4)) > ur5_WS_max ||  norm(g_start_above(1:2,4)) < ur5_WS_min ||...
       norm(      g_target(1:3,4)) > ur5_WS_max ||       norm(g_target(1:2,4)) < ur5_WS_min ||...
       norm(g_target_above(1:3,4)) > ur5_WS_max || norm(g_target_above(1:2,4)) < ur5_WS_min...
        
        fprintf('\n*** Entered locations out of UR5 workspace limit, please re-enter.\n');
    % Table collision check
    elseif g_start(3,4) <= 0 || g_target(3,4) <= 0
        fprintf('\n*** Entered locations below the table (z<0), please re-enter.\n');
    else
        % Intermediate Configuration
        g_midpoint = (g_start(1:3,4) + g_target(1:3,4))/2;
        for i = 1:3
            if round(g_midpoint(i),3) == 0
                g_midpoint(i) = g_target(i,4)/100;
            end
        end
        g_midxy_sign = sign(g_midpoint(1:2));
        if (norm(g_midpoint(1:2)) < ur5_WS_min)
            midpoint_pos = g_midpoint + [midxy_offset*g_midxy_sign; midz_offset];
            midpoint_rot = g_target(1:3,1:3);
            fprintf(['*** [Trajectory Out of UR5 Workspace]\n' ...
                     '    Trajectory from start to target position may cause UR5 collision.\n' ...
                     '    Re-route Move-And-Place task using calculated intermediate configuration.\n']);
            pause(1);
        else
            midpoint_pos = g_midpoint + [0; 0; midz_offset];
            midpoint_rot = g_target(1:3,1:3);
            fprintf(['*** [Trajectory Within UR5 Workspace]\n' ...
                     '    Trajectory from start to target position does not cause UR5 collision.\n' ...
                     '    Continue Processng Move-And-Place task.\n']);
            pause(1);
        end
        g_im = [midpoint_rot midpoint_pos; [0 0 0 1]];
        frame_im = tf_frame('base_link','frame_im',g_im); pause(1);


        % -- set UR5 to Home Configuration
        ur5.move_joints(ur5.home,5); pause(6);
        
        
        % -- USER select methods for generating control trajectory
        while true
            controlMethod = input(['\n---------- Select methods for move-and-place task: ----------\n' ...
                                   '[IK]: Inverse Kinematics\n' ...
                                   '[RR]: Resolved-rate Control using Differential Kinematics\n' ...
                                   '[TJ]: Resolved-rate Control using Transpose-Jacobian\n'],'s');
            % IK Control
            if strcmp('IK',controlMethod) == 1 || strcmp('ik',controlMethod) == 1 
                [error_start_pos, error_start_rot, error_target_pos, error_target_rot, t_duration] = ...
                    mnpIKcontrol(g_start, g_start_above, g_target, g_target_above, g_im, ur5);
                break;
            % RR Control
            elseif strcmp('RR',controlMethod) == 1 || strcmp('rr',controlMethod) == 1 
                [error_start_pos, error_start_rot, error_target_pos, error_target_rot, t_duration] = ...
                    mnpRRcontrol(g_start, g_start_above, g_target, g_target_above, g_im, ur5);
                break;
            % TJ Control
            elseif strcmp('TJ',controlMethod) == 1 || strcmp('tj',controlMethod) == 1 
                [error_start_pos, error_start_rot, error_target_pos, error_target_rot, t_duration] = ...
                    mnpTJcontrol(g_start, g_start_above, g_target, g_target_above, g_im, ur5);
                break;
            % Invalid Input
            else
            fprintf('*** Invalid control method input, please re-enter.\n')
            end
        end


        % -- Display error for the corresponding Control Methods
        fprintf(['\n---------- Errors for Start Configuration and Target Configuration ----------\n' ...
                 'error_start_pos = %0.4f [cm]\n' ...
                 'error_start_rot = %0.4f [deg]\n' ...
                 'error_target_pos = %0.4f [cm]\n' ...
                 'error_target_rot = %0.4f [deg]\n'], ...
                 error_start_pos, error_start_rot, error_target_pos, error_target_rot);


        % -- Display duration of execution for the corresponding Control Methods
        fprintf(['\n---------- Duration of execution ----------\n' ...
                 'duration = %0.4f [s]\n'], t_duration);
        
    end
end

fprintf('---------- ur5_project Terminated ----------\n');
