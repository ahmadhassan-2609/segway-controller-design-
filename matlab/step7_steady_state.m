% Step 7
% Steady-state analysis and static gain from d to xdot

clear; clc;

% Parameters (same as Steps 2-5)
R=0.35; L=0.55; ell=0.85; M=20; m=90; c1=1; c2=10; g=9.81;
J = M*L^2 + m*ell^2;

A = [0, 1, 0, 0;
     0, 0, 1, 0;
     0, (M*g*L + m*g*ell)/J, -c1/J, 0;
     0, 0, 0, -c2/(M+m)];
B = [0; 0; -1/J; 1/(R*(M+m))];
F = [0; 0; m*g/J; 0];

K = place(A, B, [-4+4i, -4-4i, -5, -6]);
Acl = A - B*K;

% Steady-state response to a unit step in d (d_ss = 1)
x_ss = -Acl \ F;

disp('Steady-state x for unit step in d:');
fprintf('  x1_ss (integral of theta) = %.4f\n', x_ss(1));
fprintf('  x2_ss (theta)             = %.4e rad\n', x_ss(2));
fprintf('  x3_ss (theta_dot)         = %.4e rad/s\n', x_ss(3));
fprintf('  x4_ss (xdot)              = %.4f m/s\n', x_ss(4));

fprintf('\nStatic gain d -> xdot: %.4f m/s per m of d\n', x_ss(4));

T_ss = -K * x_ss;
fprintf('\nSteady-state torque T_ss = %.4f N*m\n', T_ss);
fprintf('Predicted (m*g*d)        = %.4f N*m\n', m*g);