clc; clear; close all;

syms vx vz ax Ax phi

eq(1) = vx == ax * Ax * sin(phi);
eq(2) = vz == ax * Ax * cos(phi);

s = solve(eq, [Ax phi]);

% params
alpha = 10;
z_amp = 0.005;
L0 = 0.18;
vel_x = 0.75;
vel_z = 0.75;

x_amp = double(subs(s.Ax(2), [ax vx vz], [alpha vel_x vel_z]));
angle = double(subs(s.phi(2), [ax vx vz], [alpha vel_x vel_z]));

N = 500;
k = 0:N;

xp = x_amp * cos(2*pi*k/N);
zp = z_amp * sin(2*pi*k/N) - L0;

xk = xp*cos(angle) + zp*sin(angle);
zk = zp*cos(angle) - xp*sin(angle);

Lmax = 0.066 + 0.15;
Lmin = 0.15 - 0.066;
xmax = Lmax*cos(2*pi*k/N);
zmax = Lmax*sin(2*pi*k/N);
xmin = Lmin*cos(2*pi*k/N);
zmin = Lmin*sin(2*pi*k/N);

figure
hold on
plot(xk,zk)
plot(xmax, zmax)
plot(xmin, zmin)