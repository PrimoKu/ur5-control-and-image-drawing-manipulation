%% EN.530.646 RDKDC - Final Project
% _*Group 7*_


%% ROTY(pitchValue)
% accepts scalar pitch value (in radians)
% returns the corresponding 3*3 rotation matrix

function Ry = ROTY(pitchValue)

Ry = [ cos(pitchValue) 0 sin(pitchValue) ;...
       0               1 0               ;...
      -sin(pitchValue) 0 cos(pitchValue)];

end