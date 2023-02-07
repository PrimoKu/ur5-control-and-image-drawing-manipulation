%% EN.530.646 RDKDC - Final Project
% _*Group 7*_


%% 4 EULERXYZINV( )
% accepts a 3*3 rotation matrix
% returns a 3*1 vector (in radians)

function EulerXYZInv = EULERXYZINV(matrixRot)

% Recall in Homework 3, 
% we derived the explicit representation of XYZ Euler Angles as follows:
%
% $$R_{xyz}(\phi,\theta,\psi)=R_x(\phi)R_y(\theta)R_z(\psi)
% =\pmatrix{c_{\theta}c_{\psi} & -c_{\theta}s_{\psi} & s_{\theta}
% \cr c_{\phi}s_{\psi}+s_{\phi}s_{\theta}c_{\psi} 
% & c_{\phi}c_{\psi}-s_{\phi}s_{\theta}s_{\psi} & -s_{\phi}c_{\theta}
% \cr s_{\phi}s_{\psi}-c_{\phi}s_{\theta}c_{\psi} 
% & s_{\phi}c_{\psi}+c_{\phi}s_{\theta}s_{\psi} & c_{\phi}c_{\theta}}
% =\pmatrix{r_{11} & r_{12} & r_{13}
% \cr r_{21} & r_{22} & r_{23}
% \cr r_{31} & r_{32} & r_{33}}$$
%
% Where the inverse formula for the XYZ Euler angles is computed as follows:
%
% $$\phi=atan2(-r_{23},r_{33})$$
% 
% $$\theta=atan2(r_{13},\sqrt{r_{23}^2+r_{33}^2})$$
%
% $$\psi=atan2(-r_{12},r_{11})$$
%
% Notice that when deriving and solving for the inverse formula 
% $\vec{y} = [\phi;\theta;\psi]$,
% the mathmatical assumption $cos(\theta) \neq 0$ was made to ensure that 
% we can cancel the $cos(\theta)$ term in the numerator and denominator
% when deriving the inverse formula for $\phi$ and $\psi$.
% Hence, for cases where $cos(\theta) = 0$ (i.e. $\theta = \pi/2\ or\ -\pi/2$),
% they will result in numerically ill-defined $EULERINV(R)$ function.
%
% In other words, singularity occurs in the matrix when 
% the second Euler rotation $\theta = \pi/2\ or\ -\pi/2$,
% which results in having infite number of solutions to the corresponding Euler sequence.
% 
% To handle cases where singularity occurs, 
% we have the expression of $R_{xyz}(\phi,\theta,\psi)$ as follows:
%
% _Case 1: $\theta=\pi/2:$_
%
% $$R_{xyz}(\phi,\pi/2,\psi)
% =\pmatrix{0 & 0 & 1
% \cr sin(\phi+\psi) & cos(\phi+\psi) & 0
% \cr -cos(\phi+\psi) & sin(\phi+\psi) & 0}
% =\pmatrix{r_{11} & r_{12} & r_{13}
% \cr r_{21} & r_{22} & r_{23}
% \cr r_{31} & r_{32} & r_{33}}$$
%
% From the equation, we have that $\phi+\psi = atan2(r_{21},-r_{31}),
% \ and\ \phi+\psi = atan2(r_{32},r_{22})$, 
% one possible solution can then be obtained by setting $\phi=0$,
% and calculating for $\psi$ using the above equations.  
%
% _Case 2: $\theta=-\pi/2:$_
%
% $$R_{xyz}(\phi,-\pi/2,\psi)
% =\pmatrix{0 & 0 & -1
% \cr -sin(\phi-\psi) & cos(\phi-\psi) & 0
% \cr cos(\phi-\psi) & sin(\phi-\psi) & 0}
% =\pmatrix{r_{11} & r_{12} & r_{13}
% \cr r_{21} & r_{22} & r_{23}
% \cr r_{31} & r_{32} & r_{33}}$$
%
% From the equation, we have that $\phi-\psi = atan2(-r_{21},r_{31}),
% \ and\ \phi-\psi = atan2(r_{32},r_{22})$, 
% one possible solution can then be obtained by setting $\phi=0$,
% and calculating for $\psi$ using the above equations.

EulerXYZInv = [atan(-matrixRot(2,3)/matrixRot(3,3)) ...
               atan(matrixRot(1,3)/sqrt(matrixRot(2,3)^2+matrixRot(3,3)^2)) ...
               atan(-matrixRot(1,2)/matrixRot(1,1))];

if round(matrixRot(1,1),4) == 0 && round(matrixRot(1,2),4) == 0 && round(matrixRot(2,3),4) == 0 && round(matrixRot(3,3),4) == 0
    % $cos(\theta) = 0$
    % One possible solution is to set $\phi = 0$, 
    % then solve for $\psi$ with the obtained formula for different cases.
    warning(['Singularity occurs at theta = pi/2 and -pi/2 for XYZ Euler Angles. ' ...
             'One possible solution can be obtained by the method mentioned ' ...
             'in the comments at top of the section, and in the Lab Report.']);
    warning('EULERXYZINV(R) is ill-defined.');
    % Case 1: $\theta =  \pi/2$, $r_{13} =  1$.
    if round(matrixRot(1,3),4) == 1
        EulerXYZInv = [0  pi/2  atan(matrixRot(3,2)/matrixRot(2,2))];
    end
    % Case 2: $\theta = -\pi/2$, $r_{13} = -1$.
    if round(matrixRot(1,3),4) == -1
        EulerXYZInv = [0 -pi/2 -atan(matrixRot(3,2)/matrixRot(2,2))];
    end
end

end