clear all
close all
clc

3:5%j:k то же, что и [j,j+1,…,k];
9:3 %j:k пустой вектор, если j>k;
9:-2:3
2:3:10 %j:i:k то же, что и [j,j+i,j+2i,…,k];
2:-1:5 % j:i:k пустой вектор, если i>0 и j>k или если i<0 и j<k, где i, j и k –скалярные величины.

a=linspace(1,12,4)
step=a(2)-a(1)

disp([ones(1,2),log10(10)])

% Вывод строки

fprintf('Hello "World"!\n')
str='экспоненту';
fprintf('Найти %s числа 3\n',str)

e=exp(1);
fprintf('Переменная e равна %f\n',e)

x1=pi;
fprintf('Параметр 1 равен %d\n',x1);
fprintf('Параметр 1 равен %.4f\n',x1);

x2=1.333;

fprintf('Параметр 1 = %g, параметр 2= %g\n',x1,x2) 
fprintf('Параметр 1 = %.3g, параметр 2= %.1g\n',x1,x2) 

A=magic(3);
fprintf('%d %d %d \n',A) 

x=2;
fun=@(x) sqrt(3*x+pi);
fun(x)

5>8
a=5<10

y=(6<10)|(7>8)|(5*3==60/4) 
a>=8
~(a>=8)
find([1 2 3 0 0 5 0 1]) % Ищет индексы 
%ненулевых элементов
find([2 3 4]>2)

r=[8 12 9 4 23 19 10]
s=r<=10 % Проверка, какие элементы меньше
%или равны 10
t=r(s) % использование логического вектора
%для индексации массива, вектор t состоит из 
%элементов, где элементы s ненулевые
w=r(r<=10)


x=2;
if x>1
disp('Условие выполнено,x>1');
end


cube=@(x) x^3;
cube(2)


[mean,st_dev]=stat([3,4,5])


try a = dot([3,4],[1,1,1]); 
catch
    warning('Problem using function. Assigning a value of 0.'); 
    a = 0;
end

global x1;
x1=56;


%for
n=4
m=3
x=rand(1,5)
for i=1:n	
    for j=1:m
	a(i,j)=x(i);	
    end
end
a

a=randi(10,1,5)
a0=a
n=length(a)
for i=1:n-1
	for k=i+1:n
		if a(i)<a(k)
			rn=a(i);
			a(i)=a(k);
			a(k)=rn;
		end
	end
end
a0
a

%While

while true 
	n=n+1 
	if n>250 
		break 
	end
end 

%Switch
% k=['dd' 'ff' 'gg'];
k=1;
switch k  
case {1, 2}
    'good' 
case 3 
   'yes' 
otherwise
'hello' 
end

%Символьные вычисления
a=sym('a',[1 4])

syms % все переменные
%только переменная a была создана
a(1,1) % Обращение к элементам

syms A [3 3]
A
syms
% Можно напрямую обращаться к элементам
%Ai_j

h=A1_1+2*A1_1+2*A2_2+3*A3_3

syms a b c x y
f=a*x^2+b*x+c

syms 'p_a%d' 'p_b%d' [1 4]
p_a

syms x y integer 
syms z positive rational
assumptions



g=2*a/3+4*a/7-6.5*x+x/3+4*5/3-1.5
% sym a b
% a=3;
% b=5

a=sym(3) % создание символьного числа 3
b=sym(5) % создание символьного числа 5
e=b/a+sqrt(2)

c=3
d=5
f=d/c+sqrt(2)

f2=d/a+sqrt(2)

syms x
M = [x x^3; x^2 x^4];
ff(x)=M
ff(2)

syms x y
S=(x^2+x-exp(x))*(x+3)
F=collect(S)

T=(2*x^2+y^2)*(x+y^2+3)
G=collect(T) %Сначала собирает с x, потом с y

H=collect(T,y)

syms a x y
S=(x+5)*(x-a)*(x+4)
T=expand(S)

expand(sin(x-y))
expand(sin(2*x))


S=x^3+4*x^2-11*x-30
factor(S)

S=(x^2+5*x+6)/(x+2)
SA=simplify(S)

pretty(S)

%Решение алгебраических уравнений
syms a b x y z
h=solve(exp(2*z)-5)
S=x^2-x-6==0
k=solve(S)

solve(cos(2*y)+3*sin(y)-2)

solve(cos(2*y)+3*sin(y)==2)

T=a*x^2+5*b*x+20
solve(T) % относительно x
M=solve(T,a) % относительно a

% Решение систем уравнений

syms x y t
S=10*x+12*y+16*t;
[xt yt]=solve(S,5*x-y-13*t)
% x,y - первые переменные в порядке по умолчанию

[tx yx]=solve(S,5*x-y-13*t,t,y)

%Дифференцирование 
syms x y t
S=exp(x^4);
diff(S)
diff((1-4*x)^3)
R=5*y^2*cos(3*t)
diff(R)
diff(R,t)
diff(S,2) %Производная второго порядка

%Интегрирование

syms x y t
S=2*cos(x)-6*x;
int(S)
int(x*sin(x))
R=5*y^2*cos(4*t)
int(R)
int(R,t)

syms y
int(sin(y)-5*y^2,0,pi)

% ОДУ
syms y(t);
Sol=dsolve(diff(y,t)+3*y==100)
%dsolve('Dy+3*y-100')

syms y(t) a
eqn = diff(y,t,2) == a*y;
ySol(t) = dsolve(eqn)
ySol(0)

% Частное решение ОДУ
syms y(t) a
eqn = diff(y,t) == a*y;
cond = y(0) == 5;
ySol(t) = dsolve(eqn,cond)

syms y(t) 
eqn=diff(y,t,2)-2*diff(y,t)+2*y
Dy=diff(y,t);
cond=[y(0)==5;Dy(0)==0]
sol=dsolve(eqn,cond)

%Численные расчеты
syms x
S=0.8*x^3+4*exp(0.5*x)
SD=diff(S)
subs(SD,x,2) %x=2
subs(SD,x,[2:0.5:4]) %x=[2:0.5:4]


syms a b
subs(cos(a) + sin(b), [a,b], [sym('alpha'),2])

