%% EN.530.646 RDKDC - Final Project
% _*Group 7*_


%% AxisToSkew3(w)
% AxisToSkew(w) generates skew symmetric matrix given 3-vector w
% accepts a 3*1 vector w = [w1; w2; w3]
% returns the corresponding canonical 3*3 skew-symmetric matrix

function Skew3 = AxisToSkew3(x)

Skew3 = [   0  -x(3)  x(2) ;...
          x(3)    0  -x(1) ;...
         -x(2)  x(1)    0 ];

end