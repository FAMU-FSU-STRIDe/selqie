% Jonathan Boylan
clc; clear; close all

%% Forward Kinematics

syms thetaA thetaB L1 L2

alpha = 1/2 * (pi - thetaA - thetaB);
gamma = asin(L1/L2 * sin(alpha));
phi = pi - alpha - gamma;
theta = -(thetaA + alpha);
R = L2 * sin(phi) / sin(alpha);
X = R * cos(theta);
Z = R * sin(theta);

LegSpace = [X; Z];

%% Jacobian

Jacobian = jacobian(LegSpace, [thetaA; thetaB]);

l1 = 0.065;
l2 = 0.150;

J = @(A,B) double(subs(Jacobian, [thetaA thetaB L1 L2], [A B l1 l2]))

F = [0; 10];
tA = 0;
tB = 0;
Tau = J(tA, tB)' * F