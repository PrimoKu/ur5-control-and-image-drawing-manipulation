%% EN.530.646 RDKDC - Final Project
% _*Group 7*_


%% SKEW3(x)
% SkewToAxis(S) extracts vector w from skew-symmetric matrix S

function w = SkewToAxis3(S)

w = [S(3, 2); S(1, 3); S(2, 1)];

end