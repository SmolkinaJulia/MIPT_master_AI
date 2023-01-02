%hw2_15
%1
syms x y;
S = simplify((x+y)/(1/x +1/y));

%2
syms t g h;
f_1 = solve(4*t*h^2+20*t-5*g ==0,t);
f_2 = solve(4*t*h^2+20*t-5*g ==0,g);

%3
syms x y R;
eq = [(x-2)^2 + (y-4)^2 ==R^2, y == x/2 + 1];
vars = [x y];
[solx, soly] = solve(eq,vars);


