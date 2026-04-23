% Step 8
% Reduced 3-state model with full-state feedback

clear; clc;

% Parameters
R=0.35; L=0.55; ell=0.85; M=20; m=90; c1=1; c2=10; g=9.81;
J = M*L^2 + m*ell^2;

% 3-state model: x = [theta; theta_dot; xdot]
A = [0,                   1,      0;
     (M*g*L+m*g*ell)/J,  -c1/J,   0;
     0,                   0,     -c2/(M+m)];
B = [0; -1/J; 1/(R*(M+m))];
F = [0; m*g/J; 0];

% Controllability
Cctrb = ctrb(A, B);
fprintf('rank(ctrb) = %d  (need 3 for full controllability)\n\n', rank(Cctrb));

% Pole placement for ts ≈ 1 s (dominant sigma = 4)
desired_poles = [-4+4i, -4-4i, -5];
K = place(A, B, desired_poles);

disp('Gain vector K:'); disp(K);

% Verify closed-loop eigenvalues
Acl = A - B*K;
disp('Closed-loop eigenvalues of A-BK:');
disp(eig(Acl));

% Steady-state (for discussion — comparison with Step 5)
d_ss = 0.01;  % same as Step 4 hold value
x_ss = -Acl \ (F * d_ss);
fprintf('\nFor a constant d = %g m:\n', d_ss);
fprintf('  theta_ss  = %.6f rad (%.4f deg)\n', x_ss(1), x_ss(1)*180/pi);
fprintf('  xdot_ss   = %.4f m/s\n', x_ss(3));