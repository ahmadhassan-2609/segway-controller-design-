% Step 3
% PID controller design via pole placement

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

% State-space matrices (from Step 2)
A = [0, 1,                   0,        0;
     0, 0,                   1,        0;
     0, (M*g*L + m*g*ell)/J, -c1/J,    0;
     0, 0,                   0,        -c2/(M+m)];

B = [0; 0; -1/J; 1/(R*(M+m))];

F = [0; 0; m*g/J; 0];

% Controller design via pole placement
% The 4th pole is fixed at -c2/(M+m) since K(4)=0, so we place 3 poles
% on the 3x3 subsystem
A_sub = A(1:3, 1:3);
B_sub = B(1:3);

desired_poles = [-3+3i, -3-3i, -6];
K_sub = place(A_sub, B_sub, desired_poles);

K = [K_sub, 0];

disp('Gains [k1, k2, k3, 0]:'); disp(K);

% Verify closed-loop eigenvalues of the full 4x4 system
Acl = A - B*K;
disp('Closed-loop eigenvalues of A-BK:');
disp(eig(Acl));