function hw2()
% --- Система
g = 9.81;
Iyy = 1.436e-2;
% A = [ 0 0 0 0;
%       1 0 0 0;
%       0 g 0 0;
%       0 0 1 0];
% B = [1/Iyy; 0; 0; 0];
A = [0              -1.53921/41.1368    -1.53921/41.1368*0.62;
     41.1368*10^-6      0                   0;
     0              -1.5*10^-3          -1.5*10^-3];

B = [1;
     0;
     0];

% % M = 0.5;
% % m = 0.2;
% % b = 0.1;
% % I = 0.006;
% % g = 9.8;
% % l = 0.3;
% % 
% % p = I*(M+m) + M*m*l^2;
% % 
% % A = [0 1 0 0;
% %     0 -(I+m*l^2)*b/p (m^2*g*l^2)/p 0;
% %     0 0 0 1;
% %     0 -(m*l*b)/p m*g*l*(M+m)/p 0];
% % B = [0; (I+m*l^2)/p; 0; m*l/p];


% states = {'theta_dot', 'theta', 'x_dot', 'x'};
% C = eye(4);
% D = zeros(4, 1);

% --- LQR регулятор
Q = zeros(3);
% Q(2,2) = 0.001;
% Q(4,4) = 0.05;
Q(2,2) = 100;
Q(3,3) = 5000;
R = 1;
K_lqr = lqr(A, B, Q, R)
% Или можно найти:
% P = care(A, B, Q, R);
% K_lqr = R^-1*B'*P;

% --- Моделирование
% Матрицы эталонной системы
A_m = A - B*K_lqr
B_m = B;
% Время
T = 3;
tspan = 0:.03:T;
% начальное условие для рассширенной системы
% [x, x_m, vector(Theta)]'
sys_ICs = [0, -pi/8, 0, 1];
% % sys_ICs = [1, 0, -pi/8, 0];
x_m_ICs = sys_ICs;
Theta_ICs = [0 0];
ICs = [sys_ICs x_m_ICs Theta_ICs];
% задача стабилизации
g_ref = @(t) 0; 
% g_ref = @(t) sin(t);

% Траектория эталонной модели с референсной задающей функцией g(t)
sys_m = ss(A_m, B_m, eye(size(A, 1), size(A, 1)), ...
    zeros(size(B, 1), size(B, 2)));
% y = Cx + Du = x 
% значения задающего воздействия
g_ref_vals = zeros(size(tspan));
for i = 1:size(tspan, 2)
    g_ref_vals(i) = g_ref(i);
end

% вычисление переходных процессов системы
x_m = lsim(sys_m, g_ref_vals, tspan, x_m_ICs);
figure('Name', 'Reference model trajectories');
plot(tspan, x_m);
legend('theta_{dot}', 'theta', 'x_{dot}', 'x')

% интегрируем систему
[t_plot, x_cl_adept] = ode45(@(t, x) p_on_cart_RHS(t, A, B, K_lqr, ...
    A_m, B_m, x, g_ref, @MIMO_adaptive_control), tspan, ICs);
% MIMO_adaptive_control - адаптивное управление для системы с многими
% входами и выходами
figure('Name', 'Расширенная система') 
plot(t_plot, x_cl_adept);
legend('theta_{dot}', 'theta', 'x_{dot}', 'x', ...
    'theta\_m_{dot}', 'theta\_m', 'x\_m_{dot}', 'x\_m', ...
    'Theta_1', 'Theta_2')

figure('Name', 'Errors')
plot(t_plot, x_cl_adept(:, 1:4) - x_cl_adept(:, 5:8));
legend('e_1', 'e_2', 'e_3', 'e_4')


[t_plot, x_cl_adept] = ode45(@(t, x) p_on_cart_RHS(t, A, B, K_lqr, ...
    A_m, B_m, x, g_ref, @LQR_control), tspan, ICs);
% MIMO_adaptive_control - адаптивное управление для системы с многими
% входами и выходами
figure('Name', 'Расширенная система LQR') 
plot(t_plot, x_cl_adept);
legend('theta_{dot}', 'theta', 'x_{dot}', 'x', ...
    'theta\_m_{dot}', 'theta\_m', 'x\_m_{dot}', 'x\_m', ...
    'Theta_1', 'Theta_2')

figure('Name', 'Errors LQR')
plot(t_plot, x_cl_adept(:, 1:4) - x_cl_adept(:, 5:8));
legend('e_1', 'e_2', 'e_3', 'e_4')

end

% вспомогательные функции
function dxdt = p_on_cart_RHS(t, A, B, K_lqr, ...
    A_m, B_m, x, g_ref, u_ref)
    dxdt = zeros(size(x, 1), size(x, 2));
    cur_g_ref = g_ref(t);
    
    % Размерность
    n = 4; % состояние
    r = 1;% управление
    l = 2; % неопределенные параметры
    
    sys_x_ind = 1:n;
    x_m_ind = (n+1) : 2*n;
    hat_Theta_ind = (2*n+1) : (2*n + l*r);
    
   sys_x = x(sys_x_ind);
   x_m = x(x_m_ind);
   hat_Theta = reshape(x(hat_Theta_ind), r, l)';
   
   % w(x) = [-k1; -k2]' * [x1; x3]
   k1 = 5; k2 = 5;
   k1 = 0.0001; k2 = 0.0001;
   Theta = [-k1; -k2];
   Phi = [sys_x(1); sys_x(3)];
% %    Phi = [sys_x(2); sys_x(4)];
   w = Theta' * Phi;
   
   if isequal(u_ref, @MIMO_adaptive_control)
       u = MIMO_adaptive_control(K_lqr, sys_x, r, hat_Theta, Phi, cur_g_ref);
   else
       u = LQR_control(K_lqr, sys_x);
   end
   
   % Правая часть для расширенного вектора
   dxdt(sys_x_ind) = A*sys_x + B*(u + w); % для замкнутой системы
   dxdt(x_m_ind) = A_m*x_m + B_m*cur_g_ref;
   
   e = sys_x - x_m;
   d_hat_Theta = get_d_hat_Theta(n, l, A_m, B, Phi, e);
   dxdt(hat_Theta_ind) = reshape(d_hat_Theta', [], 1);
end

function u = LQR_control(K_lqr, sys_x)
    u = -K_lqr*sys_x;
end

function u = MIMO_adaptive_control(K_lqr, sys_x, r, hat_Theta, Phi, cur_g_ref)
    K_x = - K_lqr;
    K_g = eye(r);
    u = K_x*sys_x + K_g*cur_g_ref - hat_Theta'*Phi;
end

function d_hat_Theta = get_d_hat_Theta(n, l, A_m, B, Phi, e)
% (4)    
    G = eye(l);
    Q = eye(n);
    P = lyap(A_m, Q);
    
    d_hat_Theta = G*Phi*e'*P*B;
end
    