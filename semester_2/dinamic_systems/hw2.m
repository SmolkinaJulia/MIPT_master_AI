function LDUAV_adptive()

close all;
clear all;
% Матрицы системы
A = [0 -1.53921/41.1368 -1.53921/41.1368*0.62;
     41.1368*10^-6 0 0;
     0 -1.5*10^-3 -1.5*10^-3];

B = [1;0;0];
% матрица коэф лкр регулятора
Q = zeros(4, 4); % положительно полуопределенная весова матрица
Q(1, 1) = 5000;
Q(3, 3) = 100;

R = 1; % eye(2); % положительно определенная весовая матрица

P = care(A, B, Q, R); % функция вычисления уравнения Риккати
K_LQR = R^-1 * B' * P; % находим матрицу цсиления обратной связи
% матрицы эталонной системы                     
A_m = A - B * K_LQR;
B_m = B;

T = 5; 
tspan = 0:0.1:T;


sys_ICs = [1, 0, 1, 1.0];
x_m_ICs = sys_ICs;
Theta_ICs = [0, 0, 0, 0]; 
ICs = [sys_ICs x_m_ICs Theta_ICs];


g_ref = @(t) 0;
%g_ref = @(t)sin(20 * t) * cos(40 * t);


sys_m = ss(A_m, B_m, eye(size(A, 1), size(A, 1)), zeros(size(B, 1), size(B, 2)));

g_ref_vals = zeros(2, size(tspan, 2));
for i = 1:size(tspan, 2)
    g_ref_vals(1, i) = g_ref(i);
    g_ref_vals(2, i) = g_ref(i);
end

x_m = lsim(sys_m, g_ref_vals, tspan, x_m_ICs);
figure('Name', 'Reference model trajectories'); % открываем новое окно
plot(tspan, x_m);
legend('u_m', 'w_m', 'q_m', 'theta_m');

[t_plot, x_cl_adapt] = ode45(@(t, x)LDUAV_RHS(t, A, B, K_LQR, A_m, B_m, x, g_ref,...
    @MIMO_adaptive_control), tspan, ICs);

figure('Name', 'Extended system trajectories along adaptive control');
plot(t_plot, x_cl_adapt); 
legend('u', 'w', 'q', 'theta', 'u_m', 'w_m', 'q_m', 'theta_m', 'Theta_1', 'Theta_2');

figure('Name', 'Tracking errors along adaptive control');
plot(t_plot, x_cl_adapt(:, [1:4]) - x_cl_adapt(:, [5:8]));
legend('e_1', 'e_2', 'e_3', 'e_4');

[t_plot, x_cl_LQR] = ode45(@(t, x)LDUAV_RHS(t, A, B, K_LQR, A_m, B_m, x, g_ref,...
    @LQR_control), tspan, ICs);

figure('Name', 'Extended system trajectories along LQR control');
plot(t_plot, x_cl_LQR(:, [1:8]));
legend('u', 'w', 'q', 'theta', 'u_m', 'w_m', 'q_m', 'theta_m');

figure('Name', 'Tracking errors along LQR control');
plot(t_plot, x_cl_LQR(:, [1:4]) - x_cl_LQR(:, [5:8]));
legend('e_1', 'e_2', 'e_3', 'e_4');

end

function dxdt = LDUAV_RHS(t, A, B, K_LQR, A_m, B_m, x, g_ref, u_ref)
dxdt = zeros(size(x, 1), size(x, 2));
cur_g_ref = [g_ref(t); g_ref(t)];


n = 4; 
r = 2; %r = 1;
l = 2; 


sys_x_indexes = 1:n;
x_m_indexes = (n + 1):(2 * n);
hat_Theta_indexes = (2 * n + 1):(2 * n + l * r);

sys_x = x(sys_x_indexes);
x_m = x(x_m_indexes); 

hat_Theta = reshape(x(hat_Theta_indexes), r, l)';

k1 = 10; k2 = 10; k3 = 10; k4 = 10; % неизвестные коэффициеннты
Theta = [-k1, -k2; -k3, -k4];
Phi = [sys_x(2); sys_x(4)];
w = Theta' * Phi;

if isequal(u_ref, @MIMO_adaptive_control)
    u = MIMO_adaptive_control(K_LQR, sys_x, r, hat_Theta, Phi, cur_g_ref);
else
    u = LQR_control(K_LQR, sys_x);
end;


dxdt(sys_x_indexes) = A * sys_x + B * (u + w);
dxdt(x_m_indexes) = A_m * x_m + B_m * cur_g_ref;

e = sys_x - x_m;
d_hat_Theta = get_d_hat_Theta(n, l, A_m, B, Phi, e);
dxdt(hat_Theta_indexes) = reshape(d_hat_Theta', [], 1);
end

function u = LQR_control(K_LQR, sys_x)
u = -K_LQR * sys_x;
end

function u = MIMO_adaptive_control(K_LQR, sys_x, r, hat_Theta, Phi, cur_g_ref)
K_x = -K_LQR;
K_g = eye(r);
u = K_x * sys_x + K_g * cur_g_ref - hat_Theta' * Phi;
end

function d_hat_Theta = get_d_hat_Theta(n, l, A_m, B, Phi, e)
G = eye(l);
Q = eye(n);
P = lyap(A_m, Q);

d_hat_Theta = G * Phi*e'*P*B;
end
