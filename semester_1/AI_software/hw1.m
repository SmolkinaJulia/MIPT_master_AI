% homework 1 smolkina
% task 1
a_1 = (((3*1/3+2.5)/(2.5-1*1/3))...
    *((4.6-2*1/3)/(4.6+2*1/3)))/((0.05/(1/7-0.125))+5.7);
b_1 = ((5.9^2 - 2.4^2)/3)+(log10(12890)/exp(1)^0.3)^2;
c_1 = sind(80)^2-((cosd(14)*sind(80))^2)/0.18^(1/3);
% task 2
% cos(x/2)^2 = (tan(x) + sin(x))/2*tan(x)
x = pi/5;
left_part = cos(x/2)^2;
right_part = (tan(x) + sin(x))/(2*tan(x));
% left_part = right_part 
% task 3
A_3 = eye(2,2);
B_3 = ones(2,2);
C_3 = zeros(2,2);
D_3 = [A_3(:,1),C_3(:,1),A_3(:,2),C_3(:,2),...
    B_3,B_3,C_3(:,1),A_3(:,1),C_3(:,2),A_3(:,2)];
D_3 = reshape(D_3,[4,6]);
% task 4
A_4 = (reshape([36:-2:2],[6,3]))';
A_4(2,:) %elements of 2nd row
A_4(:,6) %elements of 6nd col
[A_4(3,1:2),A_4(1,end-2:end)] 
% task 5
% max(z) = max(5x1+3x2) -> min(z) = min(-5x1-3x2)
f = [-5 -3];
A = [1 1; 5 2]; % left <=
b = [4;10]; % right <=
Aeq = [];
beq = [];
lb = [0 0];
ub = [];
[x_1,x_2] = linprog(f,A,b,Aeq,beq,lb,ub);
% task 6
A_6 = [-1 2 3;4 5 -6;7 -8 9];
B_6 = [5;6;-7];
G_6 = B_6*B_6';
Q_6 = [49 56 63; 56 64 72; 63 72 81];
R_6 = 1;
[X,K,L] = icare(A_6,B_6,Q_6,R_6,[],[],G_6);

X_lyap = lyap(A_6,Q_6);
