%% EN.530.646 RDKDC - Final Project
% _*Group 7*_

 
%% TwistExp(xi, th)
% TwistExp(xi,th) gives the matrix exponential of a twist xi

% Exponentials of Twists (g)
function g = TwistExp(xi, th)
    
g = expm(TwistToHomogeneous(xi)*th);

end