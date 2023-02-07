%% EN.530.646 RDKDC - Final Project
% _*Group 7*_


%% Function manipulability(J, measure)
% 
% * Purpose:
%   Compute a measure of manipulability. 
%   Implement all three different types as defined in MLS Ch.3 Sec.4.4.
%       'sigmamin'
%       'invcond'
%       'detjac'
%   This function returns any one of the three measures of manipulability
%   as defined by the second argument.
% * Input:
%   *J*: A 6x6 matrix 
%   *measure*: A single string argument that defines 
%              which manipulability measure is used.
% * Output:
%   *mu*: The corresponding measure of manipulability.
% 

function mu = manipulability(J, measure)

% Compute A = J^T*J (J*J^T)
A = J.'*J;
% A = J*J.';

% Compute singular values (SV)
% \sigma(J) = sqrt(\lambda(A))
SV = real(sqrt(eig(A)));

% Reference: MLS Ch.3 Sec.4.4. (p.127-129)
switch measure
    % Minimum singular value of J
    % mu1 = min(\sigma(J))
    case 'sigmamin'
        mu = min(SV);
    % Inverse of the condition number of J
    % mu2 = min(\sigma(J)) / max(\sigma(J))
    case 'invcond'
        mu = min(SV) / max(SV);
    % Determinant of J
    case 'detjac'
        mu = det(J);
end

end