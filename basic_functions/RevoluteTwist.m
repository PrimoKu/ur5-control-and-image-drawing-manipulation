%% EN.530.646 RDKDC - Final Project
% _*Group 7*_


%% Revolute Twist (Xi_R)
% RevoluteTwist(q,w) gives the 6-vector corresponding to point q 
% on the axis and a screw with axis w for a revolute joint

function Xi_R = RevoluteTwist(q, w)

Xi_R = [-cross(w, q); w];

end