AB = 2;
BC = 2;
AC = 4;

h1 = 101.0;
h2 = 102.0;
h3 = 103.0;

v1 = 65.0;
v2 = 66.0;
v3 = 67.0;

h1 = deg2rad(h1);
h2 = deg2rad(h2);
h3 = deg2rad(h3);

v1 = deg2rad(v1);
v2 = deg2rad(v2);
v3 = deg2rad(v3);

cAB = sin(v1)*cos(h1)*sin(v2)*cos(h2) + sin(v1)*sin(h1)*sin(v2)*sin(h2) + cos(v1)*cos(v2);
cBC = sin(v2)*cos(h2)*sin(v3)*cos(h3) + sin(v2)*sin(h2)*sin(v3)*sin(h3) + cos(v2)*cos(v3);
cAC = sin(v1)*cos(h1)*sin(v3)*cos(h3) + sin(v1)*sin(h1)*sin(v3)*sin(h3) + cos(v1)*cos(v3);

fun = @(x) rangeEqs(x, cAB, cBC, cAC, AB, BC, AC);
x0 = [50; 51; 52];

options = optimoptions(@fsolve,'Display','iter','SpecifyObjectiveGradient',true,'MaxIterations',1000);
% options = optimoptions(@fsolve,'SpecifyObjectiveGradient',true,'MaxIterations',5000,'MaxFunctionEvaluations',5000);
% options.Algorithm = 'levenberg-marquardt';
% options.FunctionTolerance = 1e-7;

[x,F,exitflag,output,JAC] = fsolve(fun, x0, options);
% x = fsolve(fun, x0, options);