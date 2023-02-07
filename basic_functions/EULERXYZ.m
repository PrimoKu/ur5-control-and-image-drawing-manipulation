%% EN.530.646 RDKDC - Final Project
% _*Group 7*_


%% 3 EULERXYZ( )
% accepts one input - a 3-vector of angles (in radians)
% returns the corresponding 3*3 rotation matrix

function EulerXYZ = EULERXYZ(vecAngles)

EulerXYZ = ROTX(vecAngles(1))*ROTY(vecAngles(2))*ROTZ(vecAngles(3));

end