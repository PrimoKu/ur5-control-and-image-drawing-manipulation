%% EN.530.646 RDKDC - Final Project
% _*Group 7*_


%% Function getXi(g)
% 
% * Purpose:
%   Take a homogenous transformation matrix and extract the unscaled twist.
% * Input:
%   *g*: a homogeneous transformation.
% * Output:
%   *xi*: the (un-normalized) twist in 6x1 vector or twist coordinate form 
%   such that $g = exp(\hat{\xi})$
% 

function xi = getXi(g)

% Useful Variables 
zerosVec = [0; 0; 0];   % zero vector
I3 = eye(3);            % identity matrix

% Extract Rotation (skew3) part and Translation (3vector) part
R = g(1:3, 1:3);
p = g(1:3, 4);

% Compute angle of rotation (th) about axis w
th = acos((trace(R) - 1) / 2);

% -- Cases (pure translation, general)
% Pure Translation (w = 0)
if th == 0
    w = zerosVec;   % axis of rotation
    v = p/norm(p);  % translation vector
    th = norm(p);
% General (w != 0)
else
    % w^ = (R - R^T) / (2sin(th))
    w_hat = (R - R.') / (2 * sin(th));
    w = SkewToAxis3(w_hat);     % axis of rotation
                                % Check SkewToAxis3 function
    % p = [th*I3 + (1 - cos(th)) * w_hat + (th - sin(th)) * w_hat^2] * v
    % Rearranging -> v = [...]\p := A\p
    A = I3*th + (1 - cos(th)) * w_hat + (th - sin(th)) * w_hat^2;
    v = A\p;    % translation vector
end

% Construct the (un-normalized) twist
xi = [v; w]*th;

end