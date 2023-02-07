%%  drawLCSR( )
%
% * Purpose:
%   Used in drawing graphic "LCSR_logo" by the UR5
%   
%
% * Input:
%   *g_imgOrigin*: Input 4x4 transformation matrix as the desired image
%                  origin point
%
% * Output:
%   *Msg*: None


function Msg = drawLCSR(g_imgOrigin, ur5)
    %% Drawing configuration
    above_offset = 0.05;
    
    %% Detecting key features of the image and extract thresolded keypoints
    % Reading LCSR_logo.jpg & filter with thresold
    img = imread('jhulogo.png');
    gray_img = rgb2gray(img);
    filtered_img = gray_img > 230;
    
    % Using bwboundaries to detect keypoints
    feature_points = bwboundaries(filtered_img,4,'holes');
    
    % Convert keypoints to xy coordinates
    x_list = [];
    y_list = [];
    for k = 2:length(feature_points)
       edges = feature_points{k};
       x_list = [x_list; edges(:,2)];
       y_list = [y_list; edges(:,1)];
    end
    
    % Scale the image and add Proper offset for "Team5" UR5 robot
    x_list = (-min(x_list) + x_list)/(max(x_list)-min(x_list))*0.1 + g_imgOrigin(1,4);
    y_list = (-min(y_list) + y_list)/(max(y_list)-min(y_list))*0.115 + g_imgOrigin(2,4)-0.2;
    
    % Flip the image to correct direction
    x_list = -x_list;
   
    %% Print keypoints in matlab plot   
    %figure
    %scatter(x_list, y_list)
    %axis equal;

    %length(x_list)
    
    %% Arrange keypoints from same area to corresponding list
    x1_stack = []; x2_stack = []; x3_stack = [];
    y1_stack = []; y2_stack = []; y3_stack = [];
    % Bottom left area, adding origin point to make the area drawn is closed
    Area1 = [x_list(1:6:224), y_list(1:6:224);x_list(1), y_list(1)]; 
    x1_stack = [x1_stack(1:end); Area1(:,1)];
    y1_stack = [y1_stack(1:end); Area1(:,2)];
    
    % Top circle area, adding origin point to make the area drawn is closed
    Area2 = [x_list(225:6:338), y_list(225:6:338); x_list(225), y_list(225)];
    x2_stack = [x2_stack(1:end); Area2(:,1)];
    y2_stack = [y2_stack(1:end); Area2(:,2)];
    
    % Bottom right area, adding origin point to make the area drawn is closed
    Area3 = [x_list(339:6:567), y_list(339:6:567); x_list(339), y_list(339)];
    x3_stack = [x3_stack(1:end); Area3(:,1)];
    y3_stack = [y3_stack(1:end); Area3(:,2)];
    
%%  Area Figure Plot for debugging
%     figure
%     plot(x1_stack,y1_stack,'-o')
%     axis equal;
    
    %% Compute joints angles of each keypoints by Inverse Kinematics
    % Initial Configuration of Inverse Kinematics
    R = g_imgOrigin(1:4,1:3);
    p = [x1_stack(1); y1_stack(1); g_imgOrigin(3,4); 1];
    g = [R,p];
    joints_InvKin = ur5InvKin(g);
    idx = minNormIdx(joints_InvKin, ur5.home);

    % Bottom left area
    q_Area1 = zeros(6, length(x1_stack));
    for i = 1:length(x1_stack)
        g_Area1 = puretranslation(R,x1_stack(i), y1_stack(i), g_imgOrigin(3,4));
        IK_Joints1 = ur5InvKin(g_Area1);
        q_Area1(:,i) = IK_Joints1(:,idx);
    end
    % Add above offset at the start point and the end point
    Offseted_q_Area1 = TwoEndAboveOffset(q_Area1, above_offset, idx);
    
    % Top circle area
    q_Area2 = zeros(6, length(x2_stack));
    for i = 1:length(x2_stack)
        g_Area2 = puretranslation(R,x2_stack(i), y2_stack(i), g_imgOrigin(3,4));
        IK_Joints2 = ur5InvKin(g_Area2);
        q_Area2(:,i) = IK_Joints2(:,idx);
    end
    % Add above offset at the start point and the end point
    Offseted_q_Area2 = TwoEndAboveOffset(q_Area2, above_offset, idx);
    
    % Bottom right area
    q_Area3 = zeros(6, length(x3_stack));
    for i = 1:length(x3_stack)
        g_Area3 = puretranslation(R,x3_stack(i), y3_stack(i), g_imgOrigin(3,4));
        IK_Joints3 = ur5InvKin(g_Area3);
        q_Area3(:,i) = IK_Joints3(:,idx);
    end
    % Add above offset at the start point and the end point
    Offseted_q_Area3 = TwoEndAboveOffset(q_Area3, above_offset, idx);


    %% Draw the picture
    ur5.move_joints(ur5.home, 5);
    pause(6)

    Frame_idx = 0;

    for i = 1:2
        ur5.move_joints(Offseted_q_Area1(:,i),5);
        pause(5.5)
        current_joints = ur5.get_current_joints();
        pause(0.5)
        g_idx = ur5FwdKin(current_joints);
        Frame_traj = tf_frame('base_link',['traj',num2str(Frame_idx)],g_idx);
        pause(0.5)
        Frame_idx = Frame_idx + 1;
    end
    
%%
    while true
        check_attach =  input('\n---------- UR5 has been set to drawOrigin point, please attach markers. Input "Y" when done: ----------\n','s');
        if strcmp('Y', check_attach) ==  1 || strcmp('y', check_attach) ==  1
            break;
        end
    end
    
    for i = 3:length(Offseted_q_Area1)
        ur5.move_joints(Offseted_q_Area1(:,i),4);
        pause(4.5)
        current_joints = ur5.get_current_joints();
        pause(0.5)
        g_idx = ur5FwdKin(current_joints);
        Frame_traj = tf_frame('base_link',['traj',num2str(Frame_idx)],g_idx);
        pause(0.5)
        Frame_idx = Frame_idx + 1;
    end
    
    
    for i = 1:length(Offseted_q_Area2)
        ur5.move_joints(Offseted_q_Area2(:,i),4);
        pause(4.5)
        current_joints = ur5.get_current_joints();
        pause(0.5)
        g_idx = ur5FwdKin(current_joints);
        Frame_traj = tf_frame('base_link',['traj',num2str(Frame_idx)],g_idx);
        pause(0.5)
        Frame_idx = Frame_idx + 1;
    end
    
    
    for i = 1:length(Offseted_q_Area3)
        ur5.move_joints(Offseted_q_Area3(:,i),4);
        pause(4.5)
        current_joints = ur5.get_current_joints();
        pause(0.5)
        g_idx = ur5FwdKin(current_joints);
        Frame_traj = tf_frame('base_link',['traj',num2str(Frame_idx)],g_idx);
        pause(0.5)
        Frame_idx = Frame_idx + 1;
    end
    
    ur5.move_joints(ur5.home, 5);
    pause(5.5)
    
    Msg = 'done';

end