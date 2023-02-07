%% EN.530.646 RDKDC - Final Project
% _*Group 7*_

 
%% HomogeneousToTwist(xi_hat)
% HomogeneousToTwist(xi_hat) converts a homogeneous matrix to a twist

% Twist Coordinate (Xi)
function Xi = HomogeneousToTwist(xi_hat)

% Extract w_hat (skew3) part and v (3vector) part
w_hat = xi_hat(1:3, 1:3);
v = xi_hat(1:3, 4);
% Construct the twist coordinate
Xi = [v; w_hat(3, 2); w_hat(1, 3); w_hat(2, 1)];    % Vee Operation

end