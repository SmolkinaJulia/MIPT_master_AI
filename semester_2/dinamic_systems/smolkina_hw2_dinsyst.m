 function smolkina_hw2_dinsyst()
% ������� �������
A = [0              -1.53921/41.1368    -1.53921/41.1368*0.62;
     41.1368*10^-6      0                   0;
     0              -1.5*10^-3          -1.5*10^-3];

B = [1;
     0;
     0];
% ������� ���� ��� ����������
Q = zeros(3,3); % ������������ ���������������� ������ �������
Q(1, 1) = 5000;
Q(3, 3) = 100;
R = 1; 

P = care(A, B, Q, R);
K_LQR = R^-1 * B' * P ;
                      
A_m = A - B * K_LQR;
B_m = B;

T = 18; 
tspan = 0:0.1:T;

sys_ICs = [0.3, 0.01, 0.01];
x_m_ICs = sys_ICs;
Theta_ICs = [0,0]; 
ICs = [sys_ICs x_m_ICs Theta_ICs];

g_ref = @(t) 0;
% g_ref = @(t)sin(20 * t) * cos(40 * t) * tan(30 * t);

%���������� ��������� ������� � ����������
sys_m = ss(A_m, B_m, eye(size(A, 1), size(A, 1)), zeros(size(B, 1), size(B, 2)));
% 3 3 3 1

g_ref_vals = zeros(size(tspan)); % 1 51
for i = 1:size(tspan, 2)
 g_ref_vals(i) = g_ref(i);
end

% ��������� ���������� �������� �������
x_m = lsim(sys_m, g_ref_vals, tspan, x_m_ICs);
figure('Name', 'Reference model trajectories'); % ��������� ����� ����
plot(tspan, x_m);
legend('u_m', 'w_m', 'q_m', 'theta_m');

% ����������� �������
[t_plot, x_cl_adept] = ode45(@(t, x) compas_RHS(t, A, B, K_LQR, ...
    A_m, B_m, x, g_ref, @MIMO_adaptive_control), tspan, ICs);

% MIMO_adaptive_control - ���������� ���������� ��� ������� � �������
% ������� � ��������
figure('Name', '����������� �������') 
plot(t_plot, x_cl_adept);
legend('theta_{dot}', 'theta', 'x_{dot}', 'x', ...
    'theta\_m_{dot}', 'theta\_m', 'x\_m_{dot}', 'x\_m', ...
    'Theta_1', 'Theta_2')

figure('Name', 'Errors')
plot(t_plot, x_cl_adept(:, 1:4) - x_cl_adept(:, 5:8));
legend('e_1', 'e_2', 'e_3', 'e_4')


[t_plot, x_cl_adept] = ode45(@(t, x) compas_RHS(t, A, B, K_LQR, ...
    A_m, B_m, x, g_ref, @LQR_control), tspan, ICs);

% MIMO_adaptive_control - ���������� ���������� ��� ������� � �������
% ������� � ��������
figure('Name', '����������� ������� LQR') 
plot(t_plot, x_cl_adept);
legend('theta_{dot}', 'theta', 'x_{dot}', 'x', ...
    'theta\_m_{dot}', 'theta\_m', 'x\_m_{dot}', 'x\_m', ...
    'Theta_1', 'Theta_2')

figure('Name', 'Errors LQR')
plot(t_plot, x_cl_adept(:, 1:4) - x_cl_adept(:, 5:8));
legend('e_1', 'e_2', 'e_3', 'e_4')

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ��������������� �������
function dxdt = compas_RHS(t, A, B, K_LQR, ...
    A_m, B_m, x, g_ref, u_ref)
    dxdt = zeros(size(x, 1), size(x, 2));
    cur_g_ref = g_ref(t);

    % �����������
    n = 3; % ���������
    r = 1;% ����������
    l = 2; % �������������� ���������
    
    sys_x_ind = 1:n;
    x_m_ind = (n+1) : 2*n;
    hat_Theta_ind = (2*n+1) : (2*n + l*r);
    
   sys_x = x(sys_x_ind);
   x_m = x(x_m_ind);
   hat_Theta = reshape(x(hat_Theta_ind), r, l)';
   
   % w(x) = [-k1; -k2]' * [x1; x3]
   k1 = 1; k2 = 0; k3 = 0.001; % ����������� �������������
%    k1 = 0.0001; k2 = 0.0001;
   Theta = [-k1; -k2];   %; -k3];
   Phi = [sys_x(1); sys_x(3)];
%  Phi = [sys_x(2); sys_x(4)];
%    w = Theta' * Phi;
   w = Theta * Phi;
   
   if isequal(u_ref, @MIMO_adaptive_control)
       u = MIMO_adaptive_control(K_LQR, sys_x, r, hat_Theta, Phi, cur_g_ref);
   else
       u = LQR_control(K_LQR, sys_x);
   end
   
   % ������ ����� ��� ������������ �������
   dxdt(sys_x_ind) = A*sys_x + B*(u + w); % ��� ��������� �������
   dxdt(x_m_ind) = A_m*x_m + B_m*cur_g_ref;
   
   e = sys_x - x_m;
   d_hat_Theta = get_d_hat_Theta(n, l, A_m, B, Phi, e);
   dxdt(hat_Theta_ind) = reshape(d_hat_Theta', [], 1);
end

function u = LQR_control(K_LQR, sys_x)
    u = -K_LQR*sys_x;
end

function u = MIMO_adaptive_control(K_LQR, sys_x, r, hat_Theta, Phi, cur_g_ref)
    K_x = - K_LQR;
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
