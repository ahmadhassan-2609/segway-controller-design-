% Step 10 (4-state case)
% Full-state observer design

clear; clc;

% Parameters
R=0.35; L_dim=0.55; ell=0.85; M=20; m=90; c1=1; c2=10; g=9.81;
J = M*L_dim^2 + m*ell^2;

% 4-state model (Step 5 setup)
A = [0, 1, 0, 0;
     0, 0, 1, 0;
     0, (M*g*L_dim+m*g*ell)/J, -c1/J, 0;
     0, 0, 0, -c2/(M+m)];
B = [0; 0; -1/J; 1/(R*(M+m))];
F = [0; 0; m*g/J; 0];

K = place(A, B, [-4+4i, -4-4i, -5, -6]);

% Observability check
C_theta = [0 1 0 0];
fprintf('C = [0 1 0 0] (theta only): rank(obsv) = %d\n', rank(obsv(A, C_theta)));

C = eye(4);
fprintf('C = I: rank(obsv) = %d\n', rank(obsv(A, C)));

% Observer pole placement (5x faster than controller)
obs_poles = [-20+20i, -20-20i, -25, -30];
L = place(A', C', obs_poles)';

disp('Observer gain L:'); disp(L);
fprintf('Observer eigenvalues (A - L*C):\n'); disp(eig(A - L*C));