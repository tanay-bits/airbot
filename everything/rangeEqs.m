function [F,J] = rangeEqs(x, cAB, cBC, cAC, AB, BC, AC)
% Evaluate the vector function and the Jacobian matrix for 
% the system of nonlinear equations for the unknown range variables in
% the Lighthouse triangulation problem

% Evaluate the vector function
F = zeros(3,1);
F(1,1) = x(1)^2 + x(2)^2 - 2*x(1)*x(2)*cAB - AB^2;
F(2,1) = x(2)^2 + x(3)^2 - 2*x(2)*x(3)*cBC - BC^2;
F(3,1) = x(1)^2 + x(3)^2 - 2*x(1)*x(3)*cAC - AC^2;

% Evaluate the Jacobian matrix
J = [
    2*x(1)-2*x(2)*cAB, 2*x(2)-2*x(1)*cAB, 0;
    0, 2*x(2)-2*x(2)*cBC, 2*x(3)-2*x(2)*cBC;
    2*x(1)-2*x(3)*cAC, 0, 2*x(3)-2*x(1)*cAC
    ];

end

