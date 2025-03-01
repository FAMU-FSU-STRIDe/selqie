clc
clear
close all
%%

syms Vcmd Wcmd Vact Wact a b c d

eq1 = Vact == (a + b*Wcmd)*Vcmd;
eq2 = Wact == (c + d*Vcmd)*Wcmd;

s = solve([eq1 eq2], [Vcmd Wcmd]);

Vcmd = simplify(s.Vcmd)
Wcmd = simplify(s.Wcmd)

Vc = @(V,W,A,B,C,D,i) double(subs(Vcmd(i), [Vact Wact a b c d], [V W A B C D]));
Wc = @(V,W,A,B,C,D,i) double(subs(Wcmd(i), [Vact Wact a b c d], [V W A B C D]));

A = 1.0;
B = -0.3;
C = 0.125;
D = -0.2;
V = 0.15;
W = 0.1;

Vc1 = Vc(V,W,A,B,C,D,1)
Vc2 = Vc(V,W,A,B,C,D,2)
Wc1 = Wc(V,W,A,B,C,D,1)
Wc2 = Wc(V,W,A,B,C,D,2)

R = (V^2*D^2 - 2*V*W*B*D + 2*V*A*C*D + W^2*B^2 + 2*W*A*B*C + A^2*C^2)