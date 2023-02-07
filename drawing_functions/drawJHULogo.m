%% EN.530.646 RDKDC - Final Project
% _*Group 7*_


%%  drawJHULogo( )
%
% * Purpose:
%   Used in drawing graphic "jhulogo" by the UR5
%   
%
% * Input:
%   *g_imgOrigin*: Input 4x4 transformation matrix as the desired image
%                  origin point
%
% * Output:
%   *Msg*: None


function Msg = drawJHULogo(g_imgOrigin, ur5)
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

    x1_stack = []; y1_stack = [];
    start1 = 1; end1 = 420;
    Area1 = [x_list(start1:6:end1), y_list(start1:6:end1);x_list(start1), y_list(start1)]; 
    x1_stack = [x1_stack(1:end); Area1(:,1)];
    y1_stack = [y1_stack(1:end); Area1(:,2)];
    
    x2_stack = []; y2_stack = [];
    start2 = 420; end2 = 1048;
    Area2 = [x_list(start2:6:end2), y_list(start2:6:end2);x_list(start2), y_list(start2)]; 
    x2_stack = [x2_stack(1:end); Area2(:,1)];
    y2_stack = [y2_stack(1:end); Area2(:,2)];
    
    x3_stack = []; y3_stack = [];
    start3 = 1050; end3 = 1255;
    Area3 = [x_list(start3:6:end3), y_list(start3:6:end3);x_list(start3), y_list(start3)]; 
    x3_stack = [x3_stack(1:end); Area3(:,1)];
    y3_stack = [y3_stack(1:end); Area3(:,2)];

    x4_stack = []; y4_stack = [];
    start4 = 1260; end4 = 2045;
    Area4 = [x_list(start4:6:end4), y_list(start4:6:end4);x_list(start4), y_list(start4)]; 
    x4_stack = [x4_stack(1:end); Area4(:,1)];
    y4_stack = [y4_stack(1:end); Area4(:,2)];

    x5_stack = []; y5_stack = [];
    start5 = 2046; end5 = 2655;
    Area5 = [x_list(start5:6:end5), y_list(start5:6:end5);x_list(start5), y_list(start5)]; 
    x5_stack = [x5_stack(1:end); Area5(:,1)];
    y5_stack = [y5_stack(1:end); Area5(:,2)];

    x6_stack = []; y6_stack = [];
    start6 = 2660; end6 = 2725;
    Area6 = [x_list(start6:3:end6), y_list(start6:3:end6);x_list(start6), y_list(start6)]; 
    x6_stack = [x6_stack(1:end); Area6(:,1)];
    y6_stack = [y6_stack(1:end); Area6(:,2)];

    x7_stack = []; y7_stack = [];
    start7 = 2730; end7 = 2795;
    Area7 = [x_list(start7:3:end7), y_list(start7:3:end7);x_list(start7), y_list(start7)]; 
    x7_stack = [x7_stack(1:end); Area7(:,1)];
    y7_stack = [y7_stack(1:end); Area7(:,2)];

    x8_stack = []; y8_stack = [];
    start8 = 2794; end8 = 2845;
    Area8 = [x_list(start8:2:end8), y_list(start8:2:end8);x_list(start8), y_list(start8)]; 
    x8_stack = [x8_stack(1:end); Area8(:,1)];
    y8_stack = [y8_stack(1:end); Area8(:,2)];

    x9_stack = []; y9_stack = [];
    start9 = 2850; end9 = 2900;
    Area9 = [x_list(start9:2:end9), y_list(start9:2:end9);x_list(start9), y_list(start9)]; 
    x9_stack = [x9_stack(1:end); Area9(:,1)];
    y9_stack = [y9_stack(1:end); Area9(:,2)];

    x10_stack = []; y10_stack = [];
    start10 = 2905; end10 = 3145;
    Area10 = [x_list(start10:6:end10), y_list(start10:6:end10);x_list(start10), y_list(start10)]; 
    x10_stack = [x10_stack(1:end); Area10(:,1)];
    y10_stack = [y10_stack(1:end); Area10(:,2)];

    x11_stack = []; y11_stack = [];
    start11 = 3148; end11 = 3203;
    Area11 = [x_list(start11:2:end11), y_list(start11:2:end11);x_list(start11), y_list(start11)]; 
    x11_stack = [x11_stack(1:end); Area11(:,1)];
    y11_stack = [y11_stack(1:end); Area11(:,2)];

    x12_stack = []; y12_stack = [];
    start12 = 3203; end12 = 3260;
    Area12 = [x_list(start12:2:end12), y_list(start12:2:end12);x_list(start12), y_list(start12)]; 
    x12_stack = [x12_stack(1:end); Area12(:,1)];
    y12_stack = [y12_stack(1:end); Area12(:,2)];

    x13_stack = []; y13_stack = [];
    start13 = 3261; end13 = 3303;
    Area13 = [x_list(start13:2:end13), y_list(start13:2:end13);x_list(start13), y_list(start13)]; 
    x13_stack = [x13_stack(1:end); Area13(:,1)];
    y13_stack = [y13_stack(1:end); Area13(:,2)];

    x14_stack = []; y14_stack = [];
    start14 = 3305; end14 = 3348;
    Area14 = [x_list(start14:2:end14), y_list(start14:2:end14);x_list(start14), y_list(start14)]; 
    x14_stack = [x14_stack(1:end); Area14(:,1)];
    y14_stack = [y14_stack(1:end); Area14(:,2)];

    x15_stack = []; y15_stack = [];
    start15 = 3350; end15 = 3394;
    Area15 = [x_list(start15:2:end15), y_list(start15:2:end15);x_list(start15), y_list(start15)]; 
    x15_stack = [x15_stack(1:end); Area15(:,1)];
    y15_stack = [y15_stack(1:end); Area15(:,2)];

    x16_stack = []; y16_stack = [];
    start16 = 3395; end16 = 3452;
    Area16 = [x_list(start16:2:end16), y_list(start16:2:end16);x_list(start16), y_list(start16)]; 
    x16_stack = [x16_stack(1:end); Area16(:,1)];
    y16_stack = [y16_stack(1:end); Area16(:,2)];

    x17_stack = []; y17_stack = [];
    start17 = 3453; end17 = 3510;
    Area17 = [x_list(start17:2:end17), y_list(start17:2:end17);x_list(start17), y_list(start17)]; 
    x17_stack = [x17_stack(1:end); Area17(:,1)];
    y17_stack = [y17_stack(1:end); Area17(:,2)];

    x18_stack = []; y18_stack = [];
    start18 = 3511; end18 = 3554;
    Area18 = [x_list(start18:2:end18), y_list(start18:2:end18);x_list(start18), y_list(start18)]; 
    x18_stack = [x18_stack(1:end); Area18(:,1)];
    y18_stack = [y18_stack(1:end); Area18(:,2)];

    x19_stack = []; y19_stack = [];
    start19 = 3555; end19 = 3610;
    Area19 = [x_list(start19:2:end19), y_list(start19:2:end19);x_list(start19), y_list(start19)]; 
    x19_stack = [x19_stack(1:end); Area19(:,1)];
    y19_stack = [y19_stack(1:end); Area19(:,2)];

    x20_stack = []; y20_stack = [];
    start20 = 3610; end20 = 3665;
    Area20 = [x_list(start20:2:end20), y_list(start20:2:end20);x_list(start20), y_list(start20)]; 
    x20_stack = [x20_stack(1:end); Area20(:,1)];
    y20_stack = [y20_stack(1:end); Area20(:,2)];

    x21_stack = []; y21_stack = [];
    start21 = 3665; end21 = 3730;
    Area21 = [x_list(start21:2:end21), y_list(start21:2:end21);x_list(start21), y_list(start21)]; 
    x21_stack = [x21_stack(1:end); Area21(:,1)];
    y21_stack = [y21_stack(1:end); Area21(:,2)];

    x22_stack = []; y22_stack = [];
    start22 = 3732; end22 = 3799;
    Area22 = [x_list(start22:2:end22), y_list(start22:2:end22);x_list(start22), y_list(start22)]; 
    x22_stack = [x22_stack(1:end); Area22(:,1)];
    y22_stack = [y22_stack(1:end); Area22(:,2)];

    x23_stack = []; y23_stack = [];
    start23 = 3800; end23 = 4043;
    Area23 = [x_list(start23:6:end23), y_list(start23:6:end23);x_list(start23), y_list(start23)]; 
    x23_stack = [x23_stack(1:end); Area23(:,1)];
    y23_stack = [y23_stack(1:end); Area23(:,2)];

    x24_stack = []; y24_stack = [];
    start24 = 4044; end24 = 5150;
    Area24 = [x_list(start24:6:end24), y_list(start24:6:end24);x_list(start24), y_list(start24)]; 
    x24_stack = [x24_stack(1:end); Area24(:,1)];
    y24_stack = [y24_stack(1:end); Area24(:,2)];

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
    
    q_Area1 = zeros(6, length(x1_stack));
    for i = 1:length(x1_stack)
        g_Area1 = puretranslation(R,x1_stack(i), y1_stack(i), g_imgOrigin(3,4));
        IK_Joints1 = ur5InvKin(g_Area1);
        q_Area1(:,i) = IK_Joints1(:,idx);
    end
    % Add above offset at the start point and the end point
    Offseted_q_Area1 = TwoEndAboveOffset(q_Area1, above_offset, idx);
    
    q_Area2 = zeros(6, length(x2_stack));
    for i = 1:length(x2_stack)
        g_Area2 = puretranslation(R,x2_stack(i), y2_stack(i), g_imgOrigin(3,4));
        IK_Joints2 = ur5InvKin(g_Area2);
        q_Area2(:,i) = IK_Joints2(:,idx);
    end
    Offseted_q_Area2 = TwoEndAboveOffset(q_Area2, above_offset, idx);
    
    q_Area3 = zeros(6, length(x3_stack));
    for i = 1:length(x3_stack)
        g_Area3 = puretranslation(R,x3_stack(i), y3_stack(i), g_imgOrigin(3,4));
        IK_Joints3 = ur5InvKin(g_Area3);
        q_Area3(:,i) = IK_Joints3(:,idx);
    end
    Offseted_q_Area3 = TwoEndAboveOffset(q_Area3, above_offset, idx);

    q_Area4 = zeros(6, length(x4_stack));
    for i = 1:length(x4_stack)
        g_Area4 = puretranslation(R,x4_stack(i), y4_stack(i), g_imgOrigin(3,4));
        IK_Joints4 = ur5InvKin(g_Area4);
        q_Area4(:,i) = IK_Joints4(:,idx);
    end
    Offseted_q_Area4 = TwoEndAboveOffset(q_Area4, above_offset, idx);

    q_Area5 = zeros(6, length(x5_stack));
    for i = 1:length(x5_stack)
        g_Area5 = puretranslation(R,x5_stack(i), y5_stack(i), g_imgOrigin(3,4));
        IK_Joints5 = ur5InvKin(g_Area5);
        q_Area5(:,i) = IK_Joints5(:,idx);
    end
    Offseted_q_Area5 = TwoEndAboveOffset(q_Area5, above_offset, idx);

    q_Area6 = zeros(6, length(x6_stack));
    for i = 1:length(x6_stack)
        g_Area6 = puretranslation(R,x6_stack(i), y6_stack(i), g_imgOrigin(3,4));
        IK_Joints6 = ur5InvKin(g_Area6);
        q_Area6(:,i) = IK_Joints6(:,idx);
    end
    Offseted_q_Area6 = TwoEndAboveOffset(q_Area6, above_offset, idx);

    q_Area7 = zeros(6, length(x7_stack));
    for i = 1:length(x7_stack)
        g_Area7 = puretranslation(R,x7_stack(i), y7_stack(i), g_imgOrigin(3,4));
        IK_Joints7 = ur5InvKin(g_Area7);
        q_Area7(:,i) = IK_Joints7(:,idx);
    end
    Offseted_q_Area7 = TwoEndAboveOffset(q_Area7, above_offset, idx);

    q_Area8 = zeros(6, length(x8_stack));
    for i = 1:length(x8_stack)
        g_Area8 = puretranslation(R,x8_stack(i), y8_stack(i), g_imgOrigin(3,4));
        IK_Joints8 = ur5InvKin(g_Area8);
        q_Area8(:,i) = IK_Joints8(:,idx);
    end
    Offseted_q_Area8 = TwoEndAboveOffset(q_Area8, above_offset, idx);

    q_Area9 = zeros(6, length(x9_stack));
    for i = 1:length(x9_stack)
        g_Area9 = puretranslation(R,x9_stack(i), y9_stack(i), g_imgOrigin(3,4));
        IK_Joints9 = ur5InvKin(g_Area9);
        q_Area9(:,i) = IK_Joints9(:,idx);
    end
    Offseted_q_Area9 = TwoEndAboveOffset(q_Area9, above_offset, idx);

    q_Area10 = zeros(6, length(x10_stack));
    for i = 1:length(x10_stack)
        g_Area10 = puretranslation(R,x10_stack(i), y10_stack(i), g_imgOrigin(3,4));
        IK_Joints10 = ur5InvKin(g_Area10);
        q_Area10(:,i) = IK_Joints10(:,idx);
    end
    Offseted_q_Area10 = TwoEndAboveOffset(q_Area10, above_offset, idx);

    q_Area11 = zeros(6, length(x11_stack));
    for i = 1:length(x11_stack)
        g_Area11 = puretranslation(R,x11_stack(i), y11_stack(i), g_imgOrigin(3,4));
        IK_Joints11 = ur5InvKin(g_Area11);
        q_Area11(:,i) = IK_Joints11(:,idx);
    end
    Offseted_q_Area11 = TwoEndAboveOffset(q_Area11, above_offset, idx);

    q_Area12 = zeros(6, length(x12_stack));
    for i = 1:length(x12_stack)
        g_Area12 = puretranslation(R,x12_stack(i), y12_stack(i), g_imgOrigin(3,4));
        IK_Joints12 = ur5InvKin(g_Area12);
        q_Area12(:,i) = IK_Joints12(:,idx);
    end
    Offseted_q_Area12 = TwoEndAboveOffset(q_Area12, above_offset, idx);

    q_Area13 = zeros(6, length(x13_stack));
    for i = 1:length(x13_stack)
        g_Area13 = puretranslation(R,x13_stack(i), y13_stack(i), g_imgOrigin(3,4));
        IK_Joints13 = ur5InvKin(g_Area13);
        q_Area13(:,i) = IK_Joints13(:,idx);
    end
    Offseted_q_Area13 = TwoEndAboveOffset(q_Area13, above_offset, idx);

    q_Area14 = zeros(6, length(x14_stack));
    for i = 1:length(x14_stack)
        g_Area14 = puretranslation(R,x14_stack(i), y14_stack(i), g_imgOrigin(3,4));
        IK_Joints14 = ur5InvKin(g_Area14);
        q_Area14(:,i) = IK_Joints14(:,idx);
    end
    Offseted_q_Area14 = TwoEndAboveOffset(q_Area14, above_offset, idx);

    q_Area15 = zeros(6, length(x15_stack));
    for i = 1:length(x15_stack)
        g_Area15 = puretranslation(R,x15_stack(i), y15_stack(i), g_imgOrigin(3,4));
        IK_Joints15 = ur5InvKin(g_Area15);
        q_Area15(:,i) = IK_Joints15(:,idx);
    end
    Offseted_q_Area15 = TwoEndAboveOffset(q_Area15, above_offset, idx);

    q_Area16 = zeros(6, length(x16_stack));
    for i = 1:length(x16_stack)
        g_Area16 = puretranslation(R,x16_stack(i), y16_stack(i), g_imgOrigin(3,4));
        IK_Joints16 = ur5InvKin(g_Area16);
        q_Area16(:,i) = IK_Joints16(:,idx);
    end
    Offseted_q_Area16 = TwoEndAboveOffset(q_Area16, above_offset, idx);

    q_Area17 = zeros(6, length(x17_stack));
    for i = 1:length(x17_stack)
        g_Area17 = puretranslation(R,x17_stack(i), y17_stack(i), g_imgOrigin(3,4));
        IK_Joints17 = ur5InvKin(g_Area17);
        q_Area17(:,i) = IK_Joints17(:,idx);
    end
    Offseted_q_Area17 = TwoEndAboveOffset(q_Area17, above_offset, idx);

    q_Area18 = zeros(6, length(x18_stack));
    for i = 1:length(x18_stack)
        g_Area18 = puretranslation(R,x18_stack(i), y18_stack(i), g_imgOrigin(3,4));
        IK_Joints18 = ur5InvKin(g_Area18);
        q_Area18(:,i) = IK_Joints18(:,idx);
    end
    Offseted_q_Area18 = TwoEndAboveOffset(q_Area18, above_offset, idx);

    q_Area19 = zeros(6, length(x19_stack));
    for i = 1:length(x19_stack)
        g_Area19 = puretranslation(R,x19_stack(i), y19_stack(i), g_imgOrigin(3,4));
        IK_Joints19 = ur5InvKin(g_Area19);
        q_Area19(:,i) = IK_Joints19(:,idx);
    end
    Offseted_q_Area19 = TwoEndAboveOffset(q_Area19, above_offset, idx);

    q_Area20 = zeros(6, length(x20_stack));
    for i = 1:length(x20_stack)
        g_Area20 = puretranslation(R,x20_stack(i), y20_stack(i), g_imgOrigin(3,4));
        IK_Joints20 = ur5InvKin(g_Area20);
        q_Area20(:,i) = IK_Joints20(:,idx);
    end
    Offseted_q_Area20 = TwoEndAboveOffset(q_Area20, above_offset, idx);

    q_Area21 = zeros(6, length(x21_stack));
    for i = 1:length(x21_stack)
        g_Area21 = puretranslation(R,x21_stack(i), y21_stack(i), g_imgOrigin(3,4));
        IK_Joints21 = ur5InvKin(g_Area21);
        q_Area21(:,i) = IK_Joints21(:,idx);
    end
    Offseted_q_Area21 = TwoEndAboveOffset(q_Area21, above_offset, idx);

    q_Area22 = zeros(6, length(x22_stack));
    for i = 1:length(x22_stack)
        g_Area22 = puretranslation(R,x22_stack(i), y22_stack(i), g_imgOrigin(3,4));
        IK_Joints22 = ur5InvKin(g_Area22);
        q_Area22(:,i) = IK_Joints22(:,idx);
    end
    Offseted_q_Area22 = TwoEndAboveOffset(q_Area22, above_offset, idx);

    q_Area23 = zeros(6, length(x23_stack));
    for i = 1:length(x23_stack)
        g_Area23 = puretranslation(R,x23_stack(i), y23_stack(i), g_imgOrigin(3,4));
        IK_Joints23 = ur5InvKin(g_Area23);
        q_Area23(:,i) = IK_Joints23(:,idx);
    end
    Offseted_q_Area23 = TwoEndAboveOffset(q_Area23, above_offset, idx);

    q_Area24 = zeros(6, length(x24_stack));
    for i = 1:length(x24_stack)
        g_Area24 = puretranslation(R,x24_stack(i), y24_stack(i), g_imgOrigin(3,4));
        IK_Joints24 = ur5InvKin(g_Area24);
        q_Area24(:,i) = IK_Joints24(:,idx);
    end
    Offseted_q_Area24 = TwoEndAboveOffset(q_Area24, above_offset, idx);

    %% Draw the picture
    Draw_step_time = 0.3;
    Between_area_time = 3;
    print_frame_time = 0.1;
    ur5.move_joints(ur5.home, 5);
    pause(6)

    Frame_idx = 0;

    for i = 1:2
        ur5.move_joints(Offseted_q_Area1(:,i),5);
        pause(5.5)
        current_joints = ur5.get_current_joints();
        pause(print_frame_time)
        g_idx = ur5FwdKin(current_joints);
        Frame_traj = tf_frame('base_link',['traj',num2str(Frame_idx)],g_idx);
        pause(print_frame_time)
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
        if i <= 2
            ur5.move_joints(Offseted_q_Area1(:,i),Between_area_time);
            pause(Between_area_time+0.1)
        else
            ur5.move_joints(Offseted_q_Area1(:,i),Draw_step_time);
            pause(Draw_step_time+0.1)
        end
        current_joints = ur5.get_current_joints();
        pause(print_frame_time)
        g_idx = ur5FwdKin(current_joints);
        Frame_traj = tf_frame('base_link',['traj',num2str(Frame_idx)],g_idx);
        pause(print_frame_time)
        Frame_idx = Frame_idx + 1;
    end
    fprintf('-----Area1 done-----\n');
    
    
    for i = 1:length(Offseted_q_Area2)
        if i <= 2
            ur5.move_joints(Offseted_q_Area2(:,i),Between_area_time);
            pause(Between_area_time+0.1)
        else
            ur5.move_joints(Offseted_q_Area2(:,i),Draw_step_time);
            pause(Draw_step_time+0.1)
        end
        current_joints = ur5.get_current_joints();
        pause(print_frame_time)
        g_idx = ur5FwdKin(current_joints);
        Frame_traj = tf_frame('base_link',['traj',num2str(Frame_idx)],g_idx);
        pause(print_frame_time)
        Frame_idx = Frame_idx + 1;
    end
    fprintf('-----Area2 done-----\n');
    
    
    for i = 1:length(Offseted_q_Area3)
        if i <= 2
            ur5.move_joints(Offseted_q_Area3(:,i),Between_area_time);
            pause(Between_area_time+0.1)
        else
            ur5.move_joints(Offseted_q_Area3(:,i),Draw_step_time);
            pause(Draw_step_time+0.1)
        end
        current_joints = ur5.get_current_joints();
        pause(print_frame_time)
        g_idx = ur5FwdKin(current_joints);
        Frame_traj = tf_frame('base_link',['traj',num2str(Frame_idx)],g_idx);
        pause(print_frame_time)
        Frame_idx = Frame_idx + 1;
    end
    fprintf('-----Area3 done-----\n');

    for i = 1:length(Offseted_q_Area4)
        if i <= 2
            ur5.move_joints(Offseted_q_Area4(:,i),Between_area_time);
            pause(Between_area_time+0.1)
        else
            ur5.move_joints(Offseted_q_Area4(:,i),Draw_step_time);
            pause(Draw_step_time+0.1)
        end
        current_joints = ur5.get_current_joints();
        pause(print_frame_time)
        g_idx = ur5FwdKin(current_joints);
        Frame_traj = tf_frame('base_link',['traj',num2str(Frame_idx)],g_idx);
        pause(print_frame_time)
        Frame_idx = Frame_idx + 1;
    end
    fprintf('-----Area4 done-----\n');
    
    for i = 1:length(Offseted_q_Area5)
        if i <= 2
            ur5.move_joints(Offseted_q_Area5(:,i),Between_area_time);
            pause(Between_area_time+0.1)
        else
            ur5.move_joints(Offseted_q_Area5(:,i),Draw_step_time);
            pause(Draw_step_time+0.1)
        end
        current_joints = ur5.get_current_joints();
        pause(print_frame_time)
        g_idx = ur5FwdKin(current_joints);
        Frame_traj = tf_frame('base_link',['traj',num2str(Frame_idx)],g_idx);
        pause(print_frame_time)
        Frame_idx = Frame_idx + 1;
    end
    fprintf('-----Area5 done-----\n');

    for i = 1:length(Offseted_q_Area6)
        if i <= 2
            ur5.move_joints(Offseted_q_Area6(:,i),Between_area_time);
            pause(Between_area_time+0.1)
        else
            ur5.move_joints(Offseted_q_Area6(:,i),Draw_step_time);
            pause(Draw_step_time+0.1)
        end
        current_joints = ur5.get_current_joints();
        pause(print_frame_time)
        g_idx = ur5FwdKin(current_joints);
        Frame_traj = tf_frame('base_link',['traj',num2str(Frame_idx)],g_idx);
        pause(print_frame_time)
        Frame_idx = Frame_idx + 1;
    end
    fprintf('-----Area6 done-----\n');

    for i = 1:length(Offseted_q_Area7)
        if i <= 2
            ur5.move_joints(Offseted_q_Area7(:,i),Between_area_time);
            pause(Between_area_time+0.1)
        else
            ur5.move_joints(Offseted_q_Area7(:,i),Draw_step_time);
            pause(Draw_step_time+0.1)
        end
        current_joints = ur5.get_current_joints();
        pause(print_frame_time)
        g_idx = ur5FwdKin(current_joints);
        Frame_traj = tf_frame('base_link',['traj',num2str(Frame_idx)],g_idx);
        pause(print_frame_time)
        Frame_idx = Frame_idx + 1;
    end
    fprintf('-----Area7 done-----\n');
    
    for i = 1:length(Offseted_q_Area8)
        if i <= 2
            ur5.move_joints(Offseted_q_Area8(:,i),Between_area_time);
            pause(Between_area_time+0.1)
        else
            ur5.move_joints(Offseted_q_Area8(:,i),Draw_step_time);
            pause(Draw_step_time+0.1)
        end
        current_joints = ur5.get_current_joints();
        pause(print_frame_time)
        g_idx = ur5FwdKin(current_joints);
        Frame_traj = tf_frame('base_link',['traj',num2str(Frame_idx)],g_idx);
        pause(print_frame_time)
        Frame_idx = Frame_idx + 1;
    end
    fprintf('-----Area8 done-----\n');

    for i = 1:length(Offseted_q_Area9)
        if i <= 2
            ur5.move_joints(Offseted_q_Area9(:,i),Between_area_time);
            pause(Between_area_time+0.1)
        else
            ur5.move_joints(Offseted_q_Area9(:,i),Draw_step_time);
            pause(Draw_step_time+0.1)
        end
        current_joints = ur5.get_current_joints();
        pause(print_frame_time)
        g_idx = ur5FwdKin(current_joints);
        Frame_traj = tf_frame('base_link',['traj',num2str(Frame_idx)],g_idx);
        pause(print_frame_time)
        Frame_idx = Frame_idx + 1;
    end
    fprintf('-----Area9 done-----\n');

    for i = 1:length(Offseted_q_Area10)
        if i <= 2
            ur5.move_joints(Offseted_q_Area10(:,i),Between_area_time);
            pause(Between_area_time+0.1)
        else
            ur5.move_joints(Offseted_q_Area10(:,i),Draw_step_time);
            pause(Draw_step_time+0.1)
        end
        current_joints = ur5.get_current_joints();
        pause(print_frame_time)
        g_idx = ur5FwdKin(current_joints);
        Frame_traj = tf_frame('base_link',['traj',num2str(Frame_idx)],g_idx);
        pause(print_frame_time)
        Frame_idx = Frame_idx + 1;
    end
    fprintf('-----Area10 done-----\n');

    for i = 1:length(Offseted_q_Area11)
        if i <= 2
            ur5.move_joints(Offseted_q_Area11(:,i),Between_area_time);
            pause(Between_area_time+0.1)
        else
            ur5.move_joints(Offseted_q_Area11(:,i),Draw_step_time);
            pause(Draw_step_time+0.1)
        end
        current_joints = ur5.get_current_joints();
        pause(print_frame_time)
        g_idx = ur5FwdKin(current_joints);
        Frame_traj = tf_frame('base_link',['traj',num2str(Frame_idx)],g_idx);
        pause(print_frame_time)
        Frame_idx = Frame_idx + 1;
    end
    fprintf('-----Area11 done-----\n');

    for i = 1:length(Offseted_q_Area12)
        if i <= 2
            ur5.move_joints(Offseted_q_Area12(:,i),Between_area_time);
            pause(Between_area_time+0.1)
        else
            ur5.move_joints(Offseted_q_Area12(:,i),Draw_step_time);
            pause(Draw_step_time+0.1)
        end
        current_joints = ur5.get_current_joints();
        pause(print_frame_time)
        g_idx = ur5FwdKin(current_joints);
        Frame_traj = tf_frame('base_link',['traj',num2str(Frame_idx)],g_idx);
        pause(print_frame_time)
        Frame_idx = Frame_idx + 1;
    end
    fprintf('-----Area12 done-----\n');

    for i = 1:length(Offseted_q_Area13)
        if i <= 2
            ur5.move_joints(Offseted_q_Area13(:,i),Between_area_time);
            pause(Between_area_time+0.1)
        else
            ur5.move_joints(Offseted_q_Area13(:,i),Draw_step_time);
            pause(Draw_step_time+0.1)
        end
        current_joints = ur5.get_current_joints();
        pause(print_frame_time)
        g_idx = ur5FwdKin(current_joints);
        Frame_traj = tf_frame('base_link',['traj',num2str(Frame_idx)],g_idx);
        pause(print_frame_time)
        Frame_idx = Frame_idx + 1;
    end
    fprintf('-----Area13 done-----\n');
    
    for i = 1:length(Offseted_q_Area14)
        if i <= 2
            ur5.move_joints(Offseted_q_Area14(:,i),Between_area_time);
            pause(Between_area_time+0.1)
        else
            ur5.move_joints(Offseted_q_Area14(:,i),Draw_step_time);
            pause(Draw_step_time+0.1)
        end
        current_joints = ur5.get_current_joints();
        pause(print_frame_time)
        g_idx = ur5FwdKin(current_joints);
        Frame_traj = tf_frame('base_link',['traj',num2str(Frame_idx)],g_idx);
        pause(print_frame_time)
        Frame_idx = Frame_idx + 1;
    end
    fprintf('-----Area14 done-----\n');
    
    for i = 1:length(Offseted_q_Area15)
        if i <= 2
            ur5.move_joints(Offseted_q_Area15(:,i),Between_area_time);
            pause(Between_area_time+0.1)
        else
            ur5.move_joints(Offseted_q_Area15(:,i),Draw_step_time);
            pause(Draw_step_time+0.1)
        end
        current_joints = ur5.get_current_joints();
        pause(print_frame_time)
        g_idx = ur5FwdKin(current_joints);
        Frame_traj = tf_frame('base_link',['traj',num2str(Frame_idx)],g_idx);
        pause(print_frame_time)
        Frame_idx = Frame_idx + 1;
    end
    fprintf('-----Area15 done-----\n');
    
    for i = 1:length(Offseted_q_Area16)
        if i <= 2
            ur5.move_joints(Offseted_q_Area16(:,i),Between_area_time);
            pause(Between_area_time+0.1)
        else
            ur5.move_joints(Offseted_q_Area16(:,i),Draw_step_time);
            pause(Draw_step_time+0.1)
        end
        current_joints = ur5.get_current_joints();
        pause(print_frame_time)
        g_idx = ur5FwdKin(current_joints);
        Frame_traj = tf_frame('base_link',['traj',num2str(Frame_idx)],g_idx);
        pause(print_frame_time)
        Frame_idx = Frame_idx + 1;
    end
    fprintf('-----Area16 done-----\n');

    for i = 1:length(Offseted_q_Area17)
        if i <= 2
            ur5.move_joints(Offseted_q_Area17(:,i),Between_area_time);
            pause(Between_area_time+0.1)
        else
            ur5.move_joints(Offseted_q_Area17(:,i),Draw_step_time);
            pause(Draw_step_time+0.1)
        end
        current_joints = ur5.get_current_joints();
        pause(print_frame_time)
        g_idx = ur5FwdKin(current_joints);
        Frame_traj = tf_frame('base_link',['traj',num2str(Frame_idx)],g_idx);
        pause(print_frame_time)
        Frame_idx = Frame_idx + 1;
    end
    fprintf('-----Area17 done-----\n');

    for i = 1:length(Offseted_q_Area18)
        if i <= 2
            ur5.move_joints(Offseted_q_Area18(:,i),Between_area_time);
            pause(Between_area_time+0.1)
        else
            ur5.move_joints(Offseted_q_Area18(:,i),Draw_step_time);
            pause(Draw_step_time+0.1)
        end
        current_joints = ur5.get_current_joints();
        pause(print_frame_time)
        g_idx = ur5FwdKin(current_joints);
        Frame_traj = tf_frame('base_link',['traj',num2str(Frame_idx)],g_idx);
        pause(print_frame_time)
        Frame_idx = Frame_idx + 1;
    end
    fprintf('-----Area18 done-----\n');

    for i = 1:length(Offseted_q_Area19)
        if i <= 2
            ur5.move_joints(Offseted_q_Area19(:,i),Between_area_time);
            pause(Between_area_time+0.1)
        else
            ur5.move_joints(Offseted_q_Area19(:,i),Draw_step_time);
            pause(Draw_step_time+0.1)
        end
        current_joints = ur5.get_current_joints();
        pause(print_frame_time)
        g_idx = ur5FwdKin(current_joints);
        Frame_traj = tf_frame('base_link',['traj',num2str(Frame_idx)],g_idx);
        pause(print_frame_time)
        Frame_idx = Frame_idx + 1;
    end
    fprintf('-----Area19 done-----\n');

    for i = 1:length(Offseted_q_Area20)
        if i <= 2
            ur5.move_joints(Offseted_q_Area20(:,i),Between_area_time);
            pause(Between_area_time+0.1)
        else
            ur5.move_joints(Offseted_q_Area20(:,i),Draw_step_time);
            pause(Draw_step_time+0.1)
        end
        current_joints = ur5.get_current_joints();
        pause(print_frame_time)
        g_idx = ur5FwdKin(current_joints);
        Frame_traj = tf_frame('base_link',['traj',num2str(Frame_idx)],g_idx);
        pause(print_frame_time)
        Frame_idx = Frame_idx + 1;
    end
    fprintf('-----Area20 done-----\n');

    for i = 1:length(Offseted_q_Area21)
        if i <= 2
            ur5.move_joints(Offseted_q_Area21(:,i),Between_area_time);
            pause(Between_area_time+0.1)
        else
            ur5.move_joints(Offseted_q_Area21(:,i),Draw_step_time);
            pause(Draw_step_time+0.1)
        end
        current_joints = ur5.get_current_joints();
        pause(print_frame_time)
        g_idx = ur5FwdKin(current_joints);
        Frame_traj = tf_frame('base_link',['traj',num2str(Frame_idx)],g_idx);
        pause(print_frame_time)
        Frame_idx = Frame_idx + 1;
    end
    fprintf('-----Area21 done-----\n');

    for i = 1:length(Offseted_q_Area22)
        if i <= 2
            ur5.move_joints(Offseted_q_Area22(:,i),Between_area_time);
            pause(Between_area_time+0.1)
        else
            ur5.move_joints(Offseted_q_Area22(:,i),Draw_step_time);
            pause(Draw_step_time+0.1)
        end
        current_joints = ur5.get_current_joints();
        pause(print_frame_time)
        g_idx = ur5FwdKin(current_joints);
        Frame_traj = tf_frame('base_link',['traj',num2str(Frame_idx)],g_idx);
        pause(print_frame_time)
        Frame_idx = Frame_idx + 1;
    end
    fprintf('-----Area22 done-----\n');

    for i = 1:length(Offseted_q_Area23)
        if i <= 2
            ur5.move_joints(Offseted_q_Area23(:,i),Between_area_time);
            pause(Between_area_time+0.1)
        else
            ur5.move_joints(Offseted_q_Area23(:,i),Draw_step_time);
            pause(Draw_step_time+0.1)
        end
        current_joints = ur5.get_current_joints();
        pause(print_frame_time)
        g_idx = ur5FwdKin(current_joints);
        Frame_traj = tf_frame('base_link',['traj',num2str(Frame_idx)],g_idx);
        pause(print_frame_time)
        Frame_idx = Frame_idx + 1;
    end
    fprintf('-----Area23 done-----\n');

    for i = 1:length(Offseted_q_Area24)
        if i <= 2
            ur5.move_joints(Offseted_q_Area24(:,i),Between_area_time);
            pause(Between_area_time+0.1)
        else
            ur5.move_joints(Offseted_q_Area24(:,i),Draw_step_time);
            pause(Draw_step_time+0.1)
        end
        current_joints = ur5.get_current_joints();
        pause(print_frame_time)
        g_idx = ur5FwdKin(current_joints);
        Frame_traj = tf_frame('base_link',['traj',num2str(Frame_idx)],g_idx);
        pause(print_frame_time)
        Frame_idx = Frame_idx + 1;
    end
    fprintf('-----Area24 done-----\n');

    ur5.move_joints(ur5.home, 5);
    pause(5.5)
    
    Msg = 'JHU logo graph drawing done.\n';

end