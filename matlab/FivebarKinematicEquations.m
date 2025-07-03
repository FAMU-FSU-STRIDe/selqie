% Jonathan Boylan
clc; clear; close all

%% Forward Kinematics

syms thetaA thetaB YA L1 L2

alpha = 1/2 * (+pi - YA*thetaA - YA*thetaB);
gamma = asin(L1/L2 * sin(alpha));
phi = pi - alpha - gamma;
theta = 1/2 * (-pi - YA*thetaA + YA*thetaB);
R = L2 * sin(phi) / sin(alpha);
X = R * cos(theta);
Z = R * sin(theta);

LegSpace = [X; Z];

%% Jacobian

Jacobian = jacobian(LegSpace, [thetaA; thetaB]);

J = @(A,B,Y,link1,link2) double(subs(Jacobian, [thetaA thetaB YA L1 L2], [A B Y link1 link2]));

%% Expanded Jacobian

dalph_dtA = -YA*0.5;
dthet_dtA = -YA*0.5;
dalph_dtB = -YA*0.5;
dthet_dtB = +YA*0.5;

dgamm_dalph = L1/L2*cos(alpha)/sqrt(1 - (L1/L2)^2 * sin(alpha)^2);
dphii_dalph = -1;
dphii_dgamm = -1;
dRadi_dphii = L2*cos(phi)/sin(alpha);
dRadi_dalph = -L2*sin(phi)*cos(alpha)/(sin(alpha)^2);

dx_dthet = -R*sin(theta);
dx_dRadi = cos(theta);
dz_dthet = R*cos(theta);
dz_dRadi = sin(theta);

dx_dtA = dx_dRadi*(dRadi_dalph*dalph_dtA + dRadi_dphii*(dphii_dalph*dalph_dtA + dphii_dgamm*dgamm_dalph*dalph_dtA)) ...
    + dx_dthet*dthet_dtA;
dx_dtB = dx_dRadi*(dRadi_dalph*dalph_dtB + dRadi_dphii*(dphii_dalph*dalph_dtB + dphii_dgamm*dgamm_dalph*dalph_dtB)) ...
    + dx_dthet*dthet_dtB;
dz_dtA = dz_dRadi*(dRadi_dalph*dalph_dtA + dRadi_dphii*(dphii_dalph*dalph_dtA + dphii_dgamm*dgamm_dalph*dalph_dtA)) ...
    + dz_dthet*dthet_dtA;
dz_dtB = dz_dRadi*(dRadi_dalph*dalph_dtB + dRadi_dphii*(dphii_dalph*dalph_dtB + dphii_dgamm*dgamm_dalph*dalph_dtB)) ...
    + dz_dthet*dthet_dtB;

JacobianExpanded = [dx_dtA, dx_dtB;
                    dz_dtA, dz_dtB];

Jexp = @(A,B,Y,link1,link2) double(subs(JacobianExpanded, [thetaA thetaB YA L1 L2], [A B Y link1 link2]));

%% Compare Jacobians

tA = pi/4;
tB = pi/4;
l1 = 0.065;
l2 = 0.150;

J1 = J(tA,tB,-1,l1,l2)
J2 = Jexp(tA,tB,-1,l1,l2)

% J1 and J2 match exactly