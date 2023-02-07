%% EN.530.646 RDKDC - Final Project
% _*Group 7*_

 
%% RigidAdjoint(g)
% RigidAdjoint(g) gives the adjoint matrix corresponding to g

% Rigid Adjoint (Ad)
function Ad = RigidAdjoint(g)
    
R = g(1:3, 1:3);
p_hat = AxisToSkew3(g(1:3, 4));
Ad = [R p_hat*R; zeros(3) R];

end