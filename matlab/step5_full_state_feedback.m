% Step 5
% Full-state feedback via pole placement

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

B = [0; 0; -1/J; 1/(R*(M+m))];

F = [0; 0; m*g/J; 0];

% Full-state pole placement
% Dominant pair: ts = 4/sigma = 1 s  ->  sigma = 4
% Dominant: -4 +/- 4j  (zeta = 1/sqrt(2), omega_n = 4*sqrt(2))
% Non-dominant: -5, -6  (further into LHP but not too far, to keep gains modest)
desired_poles = [-4+4i, -4-4i, -5, -6];
K = place(A, B, desired_poles);

disp('Gain vector K:'); disp(K);

% Verify closed-loop eigenvalues
Acl = A - B*K;
disp('Closed-loop eigenvalues of A-BK:');
disp(eig(Acl));