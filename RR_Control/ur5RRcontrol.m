%% EN.530.646 RDKDC - Final Project
% _*Group 7*_


%% Function ur5RRControl(gdesired, K, ur5)
% 
% * Purpose:
%   Implement a discrete-time resolved rate control system by
%   iteratively implementing the following Resolved Rate control system:
%   $\vec{q}_{k+1} = \vec{q}_k - KT_{step}[J_{st}^b(\vec{q}_k)]^{-1} \vec{\xi}_k$.
%   where at each time step T_{step}, the vector $\vec{\xi}_k$ would be taken 
%   by running *getXi* on the appropriate error matrix as described in class, 
%   i.e. $\vec{\xi}_k$ is such that 
%   $exp(\hat{\xi}_k) = g_{t^*t} = g_{st^*}^{-1}g_{st}$.
% * Input:
%   *gdesired*: a homogeneous transform that is 
%   the desired end-effector pose, i.e. $g_{st^*}$.
%   *K*: the gain on the controller.
%   *ur5*: the ur5 interface object.
% * Output:
%   *finalerr*: -1 if there is a failure. 
%   If convergence is achieved, give the final positional error in [cm].
% 

function [err_pos, err_rot]  = ur5RRcontrol(gdesired, K, ur5)

% Variables 
vk_thold = 1e-3;            % norm of twist components vk [m] 
wk_thold = deg2rad(0.1);    % norm of twist components wk [rad]
mu_thold = 1e-3;            % manipulability measure threshold
                            % to determine if the system is close to a singularity

% read the current robot joint angles
q0 = ur5.get_current_joints();

% compute initial forward kinematics
g0 = ur5FwdKin(q0);

% compute initial unscaled twist
xi0 = getXi(gdesired\g0);
v0 = xi0(1:3);
w0 = xi0(4:6);

% update variables to initial value
qk = q0;
gk = g0;
xik = xi0;
vk = v0;
wk = w0;

% % ADDED FOR TESTING
% fprintf('---------- Initial iteration status  ----------\n');
% fprintf('norm(vk) = %0.4f \n', norm(vk));
% fprintf('norm(wk) = %0.4f \n', norm(wk));

idx = 0;
while (norm(vk) > vk_thold || norm(wk) > wk_thold)

    % compute Body Jacobian
    Jb = ur5BodyJacobian(qk);

    % adaptive parameter tuning
    Tstep = (xik'*(Jb*Jb')*xik) / norm(Jb*Jb'*xik);

    % -- Safety Check: system is close to a singularity
    % -> determine which joint gives singularity
    if abs(manipulability(Jb, 'detjac')) < mu_thold
        fprintf('---------- Singularity ----------\n');
        fprintf('Det(Jb) = %0.4f \n', manipulability(Jb, 'detjac'));
        % Joint 3 Singularity
        fprintf('*** Joint 3 Singularity\n');
        fprintf('Adjust joint 3 then re-route.\n');
        if (qk(3) - 0) < pi/180 || (qk(3) - pi) < pi/72 
            qk(3) = sign(qk(3))*pi/4;
        end
        % Joint 5 Singularity
        fprintf('*** Joint 5 Singularity\n');
        fprintf('Adjust joint 5 then re-route.\n');
        if (qk(5) - 0) < pi/72 || (qk(5) - pi) < pi/72 
            qk(5) = sign(qk(5))*pi/4;
        end
    end
    
    % Resolved Rate Control System using Differential Kinematics
    qk_temp = qk - K*Tstep*Jb\xik;

    % Check Tstep speed limit
    notpass = true;
    while notpass == true
        joint_v = zeros(size(qk_temp));
        joint_v(:,1) = qk_temp(:,1) - ur5.get_current_joints();
        if size(qk_temp,2)>=2
            for i=2:size(qk_temp,2)
                joint_v(:,i) = qk_temp(:,i)-qk_temp(:,i-1);
            end
        end
        if max(max(abs(joint_v)))/Tstep > pi*ur5.speed_limit
            Tstep = Tstep + 0.1;
            qk_temp = qk - K*Tstep*Jb\xik;
            fprintf('Tstep increased due to velocity over UR5 speed limit\n');
        else
            notpass = false;
        end
    end

    % wrap qk to [-pi, pi]
    qk = wrapToPi(qk_temp);

    % compute forward kinematics
    gk = ur5FwdKin(qk);         % Fwd Kin of E-E
    gk_j3 = ur5FwdKin(qk,3);    % Fwd Kin of Joint 3
    
    % -- Safety Check: table collision check [Re-Route]
    % First check: Z value from Joint 3 Fwd Kin && Joint 2 value
    if gk_j3(3,4) <= 0 || qk(2) >= 0 || qk(2) <= -pi
        qk(2) = -pi/2;
        fprintf('Table collision, re-route trajectory.\n');
    end
    % Second check: Z value from E-E Fwd Kin
    %   Joint 3 < 0 deg -> Joint 3 set to  pi/2
    %   Joint 3 > 0 deg -> Joint 3 set to -pi/2
    if gk(3,4) <= 0
        qk(3) = -sign(qk(3))*pi/4;
        qk(2) = -pi/2;
        fprintf('Table collision, re-route trajectory.\n');
    end
%     % ADDED FOR TESTING
%     fprintf('J3 Z = %0.4f \n', gk_j3(3,4));
%     fprintf('EE Z = %0.4f \n', gk(3,4));
    
    % Check dt speed limit
    dt = max(1, Tstep);  % min dt value for smooth run
    notpass = true;
    while notpass == true
        joint_v = zeros(size(qk));
        joint_v(:,1) = qk(:,1) - ur5.get_current_joints();
        if size(qk,2)>=2
            for i=2:size(qk,2)
                joint_v(:,i) = qk(:,i)-qk(:,i-1);
            end
        end
        if max(max(abs(joint_v)))/dt > pi*ur5.speed_limit
            dt = dt + Tstep;
            fprintf('dt increased due to velocity over UR5 speed limit\n');
        else
            notpass = false;
        end
    end
    
    % Move the UR5
    ur5.move_joints(qk, dt);
    pause(dt+0.1);

%     % ADDED FOR TESTING
%     fprintf('--------Current iteration (%d) status---------\n', idx);
%     fprintf('Det(Jb) = %0.4f \n', manipulability(Jb, 'detjac'));
%     fprintf('norm(vk) = %0.4f \n', norm(vk));
%     fprintf('norm(wk) = %0.4f \n', norm(wk));
%     fprintf('Tstep = %0.4f \n', Tstep);
%     fprintf('dt = %0.4f \n', dt);

    % compute unscaled twist
    xik = getXi(gdesired\gk);
    vk = xik(1:3);
    wk = xik(4:6);

    % Plot Trajectory
    % ADDED FOR TESTING
%     Frame_traj = tf_frame('base_link',['traj',num2str(idx)],gk);
    idx = idx + 1;
end

% If convergence is achieved, give the final error
t_desired = gdesired(1:3,4);
R_desired = gdesired(1:3,1:3); 
t_loc = gk(1:3,4);
R_loc = gk(1:3,1:3); 
err_pos = 100 * norm(t_loc - t_desired);
err_rot = rad2deg(sqrt(trace((R_loc - R_desired)*(R_loc - R_desired).')));

end
