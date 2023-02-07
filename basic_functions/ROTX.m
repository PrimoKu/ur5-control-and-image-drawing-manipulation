%% EN.530.646 RDKDC - Final Project
% _*Group 7*_


%% ROTX(rollValue)
% accepts scalar roll value (in radians)
% returns the corresponding 3*3 rotation matrix

function Rx = ROTX(rollValue)

Rx = [1 0               0              ;...
      0 cos(rollValue) -sin(rollValue) ;...
      0 sin(rollValue)  cos(rollValue)];

end