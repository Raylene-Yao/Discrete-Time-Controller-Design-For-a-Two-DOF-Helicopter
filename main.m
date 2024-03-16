clear; clc; close all;

%% Design specifications
zeta = 0.5;    %damping ratio, zeta >= 0.46 (option by ourselves)
ts = 15;
tr = 2;    %tr = 1.8/wn, wn >= 0.9
wn = 1;    %zeta*wn >= 0.306 (option by ourselves)
T = 0.2;    %T = tr/10##########A1

%% Pitch Channel
% G_p,zeta(s) transfer function
num_p = [37.2021];
den_p = [1 0.2830 2.7452];
Gs_p = tf(num_p, den_p)    %transfer function of Gs**********Q2
Gs_poles_p = pole(Gs_p)    %poles of Gs**********Q2

% Zero-order-hold discrete equivalent, Gz transfer function
Gz_p = c2d(Gs_p, T)    %transfer function of Gz##########A2
Gz_zeros_p = zero(Gz_p)    %zeros of Gz##########A2
Gz_poles_p = pole(Gz_p)    %poles of Gz##########A2

% Desired locations of dominant closed-loop poles
disp('Desired pole in s-domain');
s_d = -zeta*wn + wn*sqrt(1 - zeta.^2)*i    %desired poles of s-domain##########A3
disp('Desired pole in z-domain');
z_d = exp(s_d*T)    %desired poles of s-domain##########A3

% Plot with K(z) = k (doesn't work)
figure(1)
rlocus(Gz_p)
title('Root Locus with K(z) = k')
zgrid(zeta, wn*T)
axis equal

% controlSystemDesigner(Gz_p);

% z = 0.8913 + 0.1559i;
% gn = rad2deg( angle( z + 0.9812));
% gd = rad2deg( angle( z - 0.9196 - 0.3152i)) + rad2deg( angle( z - 0.9196 + 0.3152i));
% kn = rad2deg( angle( z - 0.9199 - 0.2727i)) + rad2deg( angle( z - 0.9199 + 0.2727i));
% kd = rad2deg( angle( z - 1));
% theta = 180 + gn - gd - kd + kn
% a = real(z) - imag(z)/tan(theta*pi/180)    %theta<90
% % a = real(z) - imag(z)*tan((theta-90)*pi/180)    %theta>90
% upm = abs(z - 0.9199 - 0.2727i) * abs(z - 0.9199 + 0.2727i) * 0.7236 * abs(z + 0.9812);
% downm = abs(z - 1) * abs(z - a) * abs(z - 0.9196 - 0.3152i) * abs(z - 0.9196 + 0.3152i);
% k = downm/upm

k_p = 0.272;
Kz_p = tf(k_p*[1 -1.8398 0.9206], [1 -1.1929 0.1929], T)    %transfer function of Kz##########A4
Kz_zeros_p = zero(Kz_p)    %zeros of Kz##########A4
Kz_poles_p = pole(Kz_p)    %poles of Kz##########A4
Gol_p = series(Kz_p, Gz_p);
figure(2)
rlocus(Gol_p)
title('Root Locus with K(z) = k*[(z-b)^2]/[(z-1)(z-a)]')
zgrid(zeta, wn*T)
axis equal

% Closed-loop system
Gcl_p = feedback(Gol_p, 1)    %transfer function of G_cl##########A4
Gcl_zeros_p = zero(Gcl_p)    %zeros of Gcl##########A4
Gcl_poles_p = pole(Gcl_p)    %poles of Gcl##########A4

% Step response to unit step reference input
tfinal = 15;
Rz_p = tf([1 0], [1 -1], T);
[y,t] = step(Gcl_p, tfinal);
figure(3)
plot(t,y,'*')
grid
xlabel('time (s)')
title('Step response for unit step reference input')    %plot of input##########A5
stepinfo(Gcl_p)    %design specs##########A7

% Step response to unit step disturbance
tfinal_D = 20;
Dz_p = tf([1 0], [1 -1], T);
%Gcl_D = Gcl/Kz;
Gcl_D_p = feedback(Gz_p, Kz_p);
[y,t] = step(Gcl_D_p, tfinal_D);
figure(4)
plot(t,y,'*')
grid
xlabel('time (s)')
title('Step response for unit step disturbance')    %plot of disturbance##########A6
stepinfo(Gcl_D_p)    %design specs##########A7

% Motor voltage to step reference input
Gcl_M = Gcl_p/Gz_p;
[y,t] = step(Gcl_M, tfinal);
figure(5)
plot(t,y,'*')
grid
xlabel('time (s)')
title('Motor voltage for step reference input')    %plot of motor voltage##########A8

%% Yaw Channel
% G_y,zeta(s) transfer function
num_y = [7.461];
den_y = [1 0.2701 0];
Gs_y = tf(num_y, den_y)    %transfer function of Gs**********Q2
Gs_poles_y = pole(Gs_y)    %poles of Gs**********Q2

% Zero-order-hold discrete equivalent, Gz transfer function
Gz_y = c2d(Gs_y, T)    %transfer function of Gz##########A2
Gz_zeros_y = zero(Gz_y)    %zeros of Gz##########A2
Gz_poles_y = pole(Gz_y)    %poles of Gz##########A2

% Desired locations of dominant closed-loop poles
disp('Desired pole in s-domain');
s_d = -zeta*wn + wn*sqrt(1 - zeta.^2)*i    %desired poles of s-domain##########A3
disp('Desired pole in z-domain');
z_d = exp(s_d*T)    %desired poles of s-domain##########A3

% Plot with K(z) = k (doesn't work)
figure(6)
rlocus(Gz_y)
title('Root Locus with K(z) = k')
zgrid(zeta, wn*T)
axis equal

% controlSystemDesigner(Gz_y);

% z = 0.8913 + 0.1559i;
% gn = rad2deg( angle( z + 0.9822));
% gd = rad2deg( angle( z - 1)) + rad2deg( angle( z - 0.9474));
% kn = rad2deg( angle( z - 0.9352 - 0.0693i)) + rad2deg( angle( z - 0.9352 + 0.0693i));
% kd = rad2deg( angle( z - 1));
% theta = 180 + gn - gd - kd + kn
% a = real(z) - imag(z)/tan(theta*pi/180)    %theta<90
% % a = real(z) - imag(z)*tan((theta-90)*pi/180)    %theta>90
% upm = abs(z - 0.9352 - 0.0693i) * abs(z - 0.9352 + 0.0693i) * 0.1466 * abs(z + 0.9822);
% downm = abs(z - 1) * abs(z - a) * abs(z - 1) * abs(z - 0.9474);
% k = downm/upm

k_y = 2.58;
Kz_y = tf(k_y*[1 -1.8704 0.8794], [1 -0.8014 -0.1986], T)    %transfer function of Kz##########A4
Kz_zeros_y = zero(Kz_y)    %zeros of Kz##########A4
Kz_poles_y = pole(Kz_y)    %poles of Kz##########A4
Gol_y = series(Kz_y, Gz_y);
figure(7)
rlocus(Gol_y)
title('Root Locus with K(z) = k*[(z-b)^2]/[(z-1)(z-a)]')
zgrid(zeta, wn*T)
axis equal

% Closed-loop system
Gcl_y = feedback(Gol_y, 1)    %transfer function of G_cl##########A4
Gcl_zeros_y = zero(Gcl_y)    %zeros of Gcl##########A4
Gcl_poles_y = pole(Gcl_y)    %poles of Gcl##########A4

% Step response to unit step reference input
tfinal = 15;
Rz_y = tf([1 0], [1 -1], T);
[y,t] = step(Gcl_y, tfinal);
figure(8)
plot(t,y,'*')
grid
xlabel('time (s)')
title('Step response for unit step reference input')    %plot of input##########A5
stepinfo(Gcl_y)    %design specs##########A7

% Step response to unit step disturbance
tfinal_D = 20;
Dz_y = tf([1 0], [1 -1], T);
Gcl_D_y = feedback(Gz_y, Kz_y);
[y,t] = step(Gcl_D_y, tfinal_D);
figure(9)
plot(t,y,'*')
grid
xlabel('time (s)')
title('Step response for unit step disturbance')    %plot of disturbance##########A6
stepinfo(Gcl_D_y)    %design specs##########A7

%% Coupling of pitch and yaw channels
ss(Kz_p);
ss(Kz_y);
sys_k = append(Kz_p, Kz_y);
A = [    0       1   0    0;
     -2.7451 -0.2829 0    0;
         0       0   0    1;
         0       0   0 -0.2701];
B = [   0       0  ;
     37.2021 3.5306;
        0       0  ;
      2.3892  7.461];
C = [1 0 0 0;
     0 0 1 0];
D = [0 0;
     0 0];
sys_plant = ss(A, B, C, D); 
sys_plant_d = c2d(sys_plant, T);
openloopsys_d = series(sys_k, sys_plant_d);
closeloopsys_d = feedback(openloopsys_d,eye(2));    %eye(2) generate identity matrix
figure(10)
step(closeloopsys_d)    %coupling of pitch and yaw##########B1&B2
