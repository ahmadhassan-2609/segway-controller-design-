% ME 5659 Term Project - Step 1

clear; clc; close all;

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

% Transfer functions
s = tf('s');

G_theta_T = -1 / (J*s^2 + c1*s - (M*g*L + m*g*ell));
G_theta_D = (m*g) / (J*s^2 + c1*s - (M*g*L + m*g*ell));
G_xdot_T  =  1  / (R*((M+m)*s + c2));
G_xdot_D  =  0;

disp('Theta(s)/T(s):');  G_theta_T
disp('Theta(s)/D(s):');  G_theta_D
disp('Xdot(s)/T(s):');   G_xdot_T
disp('Xdot(s)/D(s) = 0');

% Poles
poles_theta = pole(G_theta_T);
poles_xdot  = pole(G_xdot_T);

disp('Poles of theta subsystem:'); disp(poles_theta);
disp('Poles of xdot subsystem:');  disp(poles_xdot);

% Stability
all_poles = [poles_theta; poles_xdot];
if any(real(all_poles) > 0)
    disp('System is UNSTABLE (RHP pole present).');
else
    disp('System is stable.');
end

% Pole-zero map
figure;
% Shade the right-half plane
patch([0 5 5 0], [-2 -2 2 2], 'r', 'FaceAlpha', 0.1, 'EdgeColor', 'none');
hold on;
% Axes through origin
xline(0, '-', 'Color', [0.4 0.4 0.4]);
yline(0, '-', 'Color', [0.4 0.4 0.4]);
% Pole markers
plot(real(all_poles), imag(all_poles), 'bx', 'MarkerSize', 12, 'LineWidth', 2);
% Label each pole
for k = 1:length(all_poles)
    text(real(all_poles(k))+0.15, imag(all_poles(k))+0.2, ...
        sprintf('s = %.3f', all_poles(k)), 'FontSize', 10, ...
        'BackgroundColor', 'w', 'EdgeColor', [0.6 0.6 0.6]);
end
grid on;
xlim([-5 5]);
ylim([-2 2]);
xlabel('Re\{s\}  (1/s)');
ylabel('Im\{s\}  (1/s)');
title('Open-loop pole-zero map - Segway plant (NUID digit 9)');
legend({'RHP (unstable region)', '', '', 'Open-loop poles'}, 'Location', 'northwest');