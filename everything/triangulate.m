function [posMat] = triangulate(h1,h2,h3,v1,v2,v3,AB,BC,AC)

h1 = deg2rad(str2double(h1));
h2 = deg2rad(str2double(h2));
h3 = deg2rad(str2double(h3));

v1 = deg2rad(str2double(v1));
v2 = deg2rad(str2double(v2));
v3 = deg2rad(str2double(v3));

cAB = sin(v1)*cos(h1)*sin(v2)*cos(h2) + sin(v1)*sin(h1)*sin(v2)*sin(h2) + cos(v1)*cos(v2);
cBC = sin(v2)*cos(h2)*sin(v3)*cos(h3) + sin(v2)*sin(h2)*sin(v3)*sin(h3) + cos(v2)*cos(v3);
cAC = sin(v1)*cos(h1)*sin(v3)*cos(h3) + sin(v1)*sin(h1)*sin(v3)*sin(h3) + cos(v1)*cos(v3);

fun = @(x) rangeEqs(x, cAB, cBC, cAC, AB, BC, AC);
x0 = [1.26; 1.265; 1.27];

options = optimoptions(@fsolve,'SpecifyObjectiveGradient',true,'MaxIterations',5000,'MaxFunctionEvaluations',5000);

xsol = fsolve(fun, x0, options);

Ra = xsol(1);
Rb = xsol(2);
Rc = xsol(3);

Xa = Ra * sin(v1) * cos(h1);
Ya = Ra * sin(v1) * sin(h1);
Za = Ra * cos(v1);

Xb = Rb * sin(v2) * cos(h2);
Yb = Rb * sin(v2) * sin(h2);
Zb = Rb * cos(v2);

Xc = Ra * sin(v3) * cos(h3);
Yc = Ra * sin(v3) * sin(h3);
Zc = Rc * cos(v3);

% posMat = [Xa, Xb, Xc; Ya, Yb, Yc];
posMat = [Xa, Xb, Xc; Ya, Yb, Yc; Za, Zb, Zc];
end

