% Step 6
% Torque saturation analysis

clear; clc;

% Parameters (same as Step 5)
R=0.35; L=0.55; ell=0.85; M=20; m=90; c1=1; c2=10; g=9.81;
J = M*L^2 + m*ell^2;

A = [0, 1, 0, 0;
     0, 0, 1, 0;
     0, (M*g*L + m*g*ell)/J, -c1/J, 0;
     0, 0, 0, -c2/(M+m)];
B = [0; 0; -1/J; 1/(R*(M+m))];
F = [0; 0; m*g/J; 0];

K = place(A, B, [-4+4i, -4-4i, -5, -6]);

% Saturation limit (80% of Step 5 peak)
Tmax_Step5 = 141;          % observed peak from Step 5
sat_limit  = 0.80 * Tmax_Step5;
fprintf('Saturation limit: +/- %.2f N*m (80%% of %g)\n', sat_limit, Tmax_Step5);