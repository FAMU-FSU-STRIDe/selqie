clc
clear
close all
%%

syms Vcmd Wcmd Vact Wact

eq1 = Vact == (1 - 0.3*Wcmd)*Vcmd;
eq2 = Wact == (1 - 1.6*Vcmd)*Wcmd/8;

s = solve([eq1 eq2], [Vcmd Wcmd]);

Vcmd = simplify(s.Vcmd)
Wcmd = simplify(s.Wcmd)

Vc = @(Vdes,Wdes,i) double(subs(Vcmd(i), [Vact, Wact], [Vdes, Wdes]));
Wc = @(Vdes,Wdes,i) double(subs(Wcmd(i), [Vact, Wact], [Vdes, Wdes]));

