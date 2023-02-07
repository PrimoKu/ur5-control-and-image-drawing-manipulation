%% EN.530.646 RDKDC - Final Project
% _*Group 7*_


%% TwistToHomogeneous(xi)
% TwistToHomogeneous(xi) converts xi from a 6 vector to a 4X4 matrix

% Homogeneous Representation of Twist (Xi_Hat)
function Xi_Hat = TwistToHomogeneous(xi)
    
Xi_Hat = [AxisToSkew3(xi(4:6)) xi(1:3); zeros(1,4)];

end