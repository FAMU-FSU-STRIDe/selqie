clc; clear; close all;

syms vx vz ax Ax phi

eq(1) = vx == -ax * Ax * sin(phi);
eq(2) = vz == ax * Ax * cos(phi);

s = solve(eq, [Ax phi])