clear all; 
clc;

% Sample time
Ts = 0.04; % period of estimator (KF) = 0.04!

% System parameters (update these values as needed)
m = 1.450;               % Mass [kg]
r = 0.0335;            % Wheel radius [m]
L = 0.157;             % Wheel base [m]
Length = 0.194;             % Robot length [m]
I = (1/12) * m * L^2;  % Moment of inertia [kg*m^2]
a = Length;
b = L;
I = (1/12) * m * (a^2 + b^2);
mu_v = 0.008;          % Coefficient of viscous damping

%% Forward
clc;

% State-space representation
A = [-2*mu_v/(m*r^2), 0;
     0, -L^2*mu_v/(I*r^2)];
B = [1/(m*r),  0;
     0,   L/(I*r)];
C = eye(2);  
D = zeros(2,2);

sysc = ss(A, B, C, D);
sysd = c2d(sysc, Ts, 'zoh');

% Augment the system with integral states
A_aug = [A, zeros(2);
         -C, zeros(2)];
B_aug = [B;
         zeros(2,2)];
C_aug = [C, zeros(2)];
D_aug = D;

sysc_aug = ss(A_aug, B_aug, C_aug, D_aug);
sysd_aug = c2d(sysc_aug, Ts, 'zoh');

% Define LQR weights
Q_c = [20  0;  % Continuous Q matrix
       0  10];

R_c = [0.2  0;  % Continuous R matrix
       0  0.5];

% Define augmented LQR weights
Q_aug_c = [20 0 0 0;  % Continuous Q matrix
           0 10 0 0;
           0 0 1 0;
           0 0 0 1];

R_aug_c = [0.1  0;  % Continuous R matrix
           0  0.25];

% Compute LQR gain
[K_c, s_c, e_c] = lqr(A, B, Q_c, R_c);
[K_d, s_d, e_d] = lqrd(A, B, Q_c, R_c, Ts);

[K_aug_c, s_aug_c, e_aug_c] = lqr(A_aug, B_aug, Q_aug_c, R_aug_c);
[K_aug_d, s_aug_d, e_aug_d] = lqrd(A_aug, B_aug, Q_aug_c, R_aug_c, Ts);
  
% Compute reference gain 
Ref = inv(inv((B*K_c - A)) * B);
Ref_aug = inv(inv((B*K_aug_c(1:2 , 1:2 , 1) - A)) * B);

Ref_d = inv((eye(size(A)) - A + B*K_d)) * B;
Ref_aug_d = inv((eye(2) - A + B*K_aug_d(1:2 , 1:2))) * B;


% Display results
disp('Continuous-time LQR gain K_c:');
disp(K_c);

disp('Discrete-time LQR gain K_d:');
disp(K_d);

%disp('Reference gain:');
%disp(Ref);

K = K_aug_d;
K_r = Ref_aug_d;

disp('K:');
disp(K)

disp('K_r:');
disp(K_r)

%% Turn only
clc;

% Reduced model for pure rotation
A_w = -L^2 * mu_v / (I * r^2);
B_w = L / (I * r);
C_w = 1;
D_w = 0;

sysc_w = ss(A_w, B_w, C_w, D_w);
sysd_w = c2d(sysc_w, Ts, 'zoh');

% Augment with integral of error
A_w_aug = [A_w, 0;
           -1, 0];
B_w_aug = [B_w;
           0];
C_w_aug = [1, 0]; % Output is still omega
D_w_aug = 0;

sysc_w_aug = ss(A_w_aug, B_w_aug, C_w_aug, D_w_aug);
sysd_w_aug = c2d(sysc_w_aug, Ts);

% Define turn LQR Weights
Qw_c = 1.0; 
Rw_c = 1.0; 

Qw_aug_c = diag([2.0, 1.0]);  
Rw_aug_c = 1.0;            

% Compute LQR gain
[K_w_c, S_w_c, e_w_c] = lqr(A_w, B_w, Qw_c, Rw_c);
[K_w_d, S_w_d, e_w_d] = lqrd(A_w, B_w, Qw_c, Rw_c, Ts);

[K_w_aug_c, S_w_aug_c, e_w_aug_c] = lqr(A_w_aug, B_w_aug, Qw_aug_c, Rw_aug_c);
[K_w_aug_d, S_w_aug_d, e_w_aug_d] = lqrd(A_w_aug, B_w_aug, Qw_aug_c, Rw_aug_c, Ts);

% Compute reference gain
Ref_w = inv(inv(B_w * K_w_c - A_w) * B_w);

Ref_w_d = inv((1 - A_w + B_w * K_w_d) * B_w);

% disp('Reference gain for rotation only (Ref_w_d):');
% disp(Ref_w_d);

disp('Continuous-time LQR gain for rotation only (K_w_c):');
disp(K_w_c);

disp('Discrete-time LQR gain for rotation only (K_w_d):');
disp(K_w_d);

disp("Continuous-time LQR gain for augmented rotation only (K_aug_w_c):\n")
disp(K_w_aug_c)

disp("Discrete-time LQR gain for augmented rotation only (K_aug_w_d):\n")
disp(K_w_aug_d)

K = K_w_aug_d;
K_r = Ref_w_d;

disp('K_omega:');
disp(K)

disp('K_r_omega:');
disp(K_r)

%% Check controllability and observability
fprintf("=== Original System ===\n");
fprintf("eig A: %.6f\n", eig(A));   
fprintf("Controllability rank: %.2f\n", rank(ctrb(A,B))); 
fprintf("Observability rank: %.2f\n", rank(obsv(A,C))); 
fprintf("\n");

fprintf("eig A_d: %.6f\n", eig(sysd.A));   
fprintf("Controllability rank (discrete): %.2f\n", rank(ctrb(sysd.A, sysd.B))); 
fprintf("Observability rank (discrete): %.2f\n", rank(obsv(sysd.A, sysd.C))); 
fprintf("\n");

fprintf("=== Augmented System ===\n");
fprintf("eig A_aug: %.6f\n", eig(A_aug));   
fprintf("Controllability rank (augmented): %.2f\n", rank(ctrb(A_aug,B_aug))); 
fprintf("Observability rank (augmented): %.2f\n", rank(obsv(A_aug,C_aug)));
fprintf("\n");

fprintf("eig A_aug_d: %.6f\n", eig(sysd_aug.A));   
fprintf("Controllability rank (augmented discrete): %.2f\n", rank(ctrb(sysd_aug.A, sysd_aug.B))); 
fprintf("Observability rank (augmented discrete): %.2f\n", rank(obsv(sysd_aug.A, sysd_aug.C))); 
fprintf("\n");

fprintf("=== Turn-only (rotation) System ===\n");
fprintf("eig A_w: %.6f\n", eig(A_w));   
fprintf("Controllability rank: %.2f\n", rank(ctrb(A_w,B_w))); 
fprintf("Observability rank: %.2f\n", rank(obsv(A_w,1))); % C = 1 for pure omega
fprintf("\n");

fprintf("eig A_w_d: %.6f\n", eig(sysd_w.A));   
fprintf("Controllability rank (discrete): %.2f\n", rank(ctrb(sysd_w.A, sysd_w.B))); 
fprintf("Observability rank (discrete): %.2f\n", rank(obsv(sysd_w.A, sysd_w.C)));
fprintf("\n");

fprintf("\n=== Augmented Turn-only System ===\n");
fprintf("eig(A_w_aug): %.6f\n", eig(A_w_aug));   
fprintf("Controllability rank (augmented): %.0f\n", rank(ctrb(A_w_aug,B_w_aug))); 
fprintf("Observability rank (augmented): %.0f\n", rank(obsv(A_w_aug,C_w_aug))); 

fprintf("\n=== Augmented Turn-only System (Discrete) ===\n");
fprintf("eig(A_w_aug_d): %.6f\n", eig(sysd_w_aug.A));   
fprintf("Controllability rank (augmented discrete): %.0f\n", rank(ctrb(sysd_w_aug.A, sysd_w_aug.B))); 
fprintf("Observability rank (augmented discrete): %.0f\n", rank(obsv(sysd_w_aug.A, sysd_w_aug.C))); 

