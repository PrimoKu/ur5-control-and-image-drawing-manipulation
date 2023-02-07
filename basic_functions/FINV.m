%% EN.530.646 RDKDC - Final Project
% _*Group 7*_


%% FINV(matrixHomo)
% accepts a 4*4 homogeneous transformation
% returns its MATRIX inverse

function Finv = FINV(matrixHomo)

R = matrixHomo(1:3,1:3);    % R
Rt = transpose(R);          % R^T
v = matrixHomo(1:3,4);      % V

Finv = eye(4);              % Accounts for Finv(4,4) = 1
Finv(1:3,1:3) = Rt;         % R^T
Finv(1:3,4) = -Rt*v;        % R^T * v

end