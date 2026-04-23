% Step 2
% State-space form, controllability, output controllability

clear; clc;

% Parameters
R   = 0.35;
L   = 0.55;
ell = 0.85;
M   = 20;
m   = 90;
c1  = 1;
c2  = 10;
g   = 9.81;

J = M*L^2 + m*ell^2;

% State-space matrices
A = [0, 1,                   0,        0;
     0, 0,                   1,        0;
     0, (M*g*L + m*g*ell)/J, -c1/J,    0;
     0, 0,                   0,        -c2/(M+m)];

B = [0;
     0;
     -1/J;
     1/(R*(M+m))];

F = [0;
     0;
     m*g/J;
     0];

C = [0 1 0 0;
     0 0 0 1];

disp('A ='); disp(A);
disp('B ='); disp(B);
disp('F ='); disp(F);
disp('C ='); disp(C);

% State controllability
Cctrb = ctrb(A, B);
disp('Controllability matrix ='); disp(Cctrb);
fprintf('rank(ctrb) = %d  (need 4 for full controllability)\n\n', rank(Cctrb));

% Output controllability
Coc = C * Cctrb;
disp('Output controllability matrix ='); disp(Coc);
fprintf('rank(output ctrb) = %d  (need 2 for output controllability)\n', rank(Coc));