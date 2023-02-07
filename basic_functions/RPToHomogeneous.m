%% EN.530.646 RDKDC - Final Project
% _*Group 7*_


%% RPToHomogeneous(R,P)
% RPToHomogeneous(R,p) forms homogeneous matrix 
% from rotation matrix R and position vector p

% Homogeneous Transformation Matrix (g_homo)
function g_homo = RPToHomogeneous(R,P)

g_homo = [R P; zeros(1,3) 1];

end