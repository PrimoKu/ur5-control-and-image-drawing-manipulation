%% EN.530.646 RDKDC - Final Project
% _*Group 7*_


%%  puretranslation( )
% * Purpose:
%   Used in drawing graphic by the UR5 to keep certain orientation.
% * Input:
%   *R*: 4x3 matrix which contains the specific rotation matrix
%       i.e. [ROTX(pi/6); zeros(1,3)]
%   *x, y, z*: the tranlation part which is computed by Inverse Kinematics
%
% * Output:
%   *g_puretranslation*: a transformation matrix with unchanged rotation
%                        with respect to the previous point.
% 

function g_puretranslation = puretranslation(R, x, y, z)

    g_puretranslation = [R, [x; y; z; 1]];

end