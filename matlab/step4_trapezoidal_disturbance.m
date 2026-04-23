% Step 4
% Trapezoidal disturbance profile and velocity simulation

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

% State-space matrices and controller (from Steps 2-3)
A = [0, 1,                   0,        0;
     0, 0,                   1,        0;
     0, (M*g*L + m*g*ell)/J, -c1/J,    0;
     0, 0,                   0,        -c2/(M+m)];

B = [0; 0; -1/J; 1/(R*(M+m))];

F = [0; 0; m*g/J; 0];

A_sub = A(1:3, 1:3);
B_sub = B(1:3);
K_sub = place(A_sub, B_sub, [-3+3i, -3-3i, -6]);
K = [K_sub, 0];

% Static-gain prediction for trapezoidal disturbance
d_peak = 0.01;
xdot_ss_predicted = m*g*d_peak / (R*c2);
fprintf('Static-gain prediction for d = %.3f m:\n', d_peak);
fprintf('  xdot_ss = %.4f m/s  (constraint: < 3 m/s)\n', xdot_ss_predicted);
fprintf('  Max d for xdot < 3 m/s: d_max = %.4f m\n', 3*R*c2/(m*g));