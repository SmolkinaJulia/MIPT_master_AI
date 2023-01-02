%hw2_16
%1
syms x(t);
dsolve(diff(x,t,2)+2*diff(x,t)+x==0)

%2 it was not obvious if s(a) or a(s), i choosed:
syms s(a) x;
eq = diff(s,a)==a*x;
s_solve(a)=dsolve(eq)

%3

syms y(t);
eq = diff(y,t)+4*y==60;
%Dy = diff(y,t);
cond = [y(0)==5];
sol = dsolve(eq,cond)



