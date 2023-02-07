%% EN.530.646 RDKDC - Final Project
% _*Group 7*_


%% ROTZ(yawValue)
% accepts scalar yaw value (in radians)
% returns the corresponding 3*3 rotation matrix

function Rz = ROTZ(yawValue)

Rz = [cos(yawValue) -sin(yawValue) 0 ;...
      sin(yawValue)  cos(yawValue) 0 ;...
      0              0             1];

end