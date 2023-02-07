%% EN.530.646 RDKDC - Final Project
% _*Group 7*_


%% Function ur5FwdKin(q)
% 
% * Purpose:
%   Compute the forward kinematic map of the UR5.
% * Input:
%   *q*: 6x1 joint space variable vector 
%   $\ =[\theta_1,\theta_2,\theta_3,\theta_4,\theta_5,\theta_6]^T$, 
%   where $\theta_n$ is the angle of joint n for $n = [1,6]$.
% * Output:
%   *gst*: End effector pose, $g_{st} \in R^{4\times4}$.
% 

function gst = ur5FwdKin(q, varargin)

% Useful Variables 
zerosVec = [0; 0; 0];   % zero vector
e1 = [1; 0; 0];         % basis vector x
e2 = [0; 1; 0];         % basis vector y
e3 = [0; 0; 1];         % basis vector z
I3 = eye(3);            % identity matrix

% *Link Lengths* Provided in PS6
% Numeric [m]
L0 = 0.0892;
L1 = 0.425;
L2 = 0.392;
L3 = 0.1093;
L4 = 0.09475;
L5 = 0.0825;

% *[q]: Positions of Axes*
% Final Project
if length(varargin) == 1
    % at this point an optional argument has been passed in 
    q0_num = varargin{1};
    switch q0_num
        case 1
            q0 = [0; 0; L0];
        case 2
            q0 = [0; L3; L0];
        case 3
            q0 = [L1; L3; L0];
        case 4
            q0 = [L1 + L2; 0; L0];
        case 5
            q0 = [L1 + L2; L3; L0];
        case 6
            q0 = [L1 + L2; L3 + L5; L0 - L4];
        otherwise
            error("Invalid Optional Argument");
    end
else
    q0 = [L1 + L2; L3 + L5; L0 - L4];
end


% q0 = [L1 + L2; L3 + L5; L0 - L4];
q1 = zerosVec;
q2 = [0; 0; L0];
q3 = [L1; 0; L0];
q4 = [L1 + L2; 0; L0];
q5 = [L1 + L2; L3; 0];
q6 = [L1 + L2; 0; L0 - L4];

% *[w] Directions of Axes* 
% Final Project
w1 =  e3;
w2 =  e2;
w3 =  e2;
w4 =  e2;
w5 = -e3;
w6 =  e2;

% *[xi] Twist Coordinates*
xi1 = RevoluteTwist(q1, w1);
xi2 = RevoluteTwist(q2, w2);
xi3 = RevoluteTwist(q3, w3);
xi4 = RevoluteTwist(q4, w4);
xi5 = RevoluteTwist(q5, w5);
xi6 = RevoluteTwist(q6, w6);

% *[gst(0)] Initial Configuration
% Transformation from initial to tool frame
% g0 = RPToHomogeneous(I3,q0);
% Final Project
g0 = RPToHomogeneous(ROTX(-pi/2) * ROTZ(pi), q0);

% *[e^{\hat{\xi}\theta}] Exponentials of Twists*
g1 = TwistExp(xi1, q(1));
g2 = TwistExp(xi2, q(2));
g3 = TwistExp(xi3, q(3));
g4 = TwistExp(xi4, q(4));	
g5 = TwistExp(xi5, q(5));
g6 = TwistExp(xi6, q(6));

% *[gst(\theta)] Forward Kinematics*
if length(varargin) == 1
    % at this point an optional argument has been passed in 
    joint_num = varargin{1};
    switch joint_num
        case 1
            gst = g1*g0;
        case 2
            gst = g1*g2*g0;
        case 3
            gst = g1*g2*g3*g0;
        case 4
            gst = g1*g2*g3*g4*g0;
        case 5
            gst = g1*g2*g3*g4*g5*g0;
        case 6
            gst = g1*g2*g3*g4*g5*g6*g0;
        otherwise
            error("Invalid Optional Argument");
    end
else
    gst = g1*g2*g3*g4*g5*g6*g0;
end

end