%% Limpando o Workspace

clear
close all;
clc

%% Declaração das variáveis do sistema
% ==============================
% Dados gerais
% ==============================
dados.geral.g = 9.81;           % [m/s^2] Aceleração da gravidade
dados.geral.Tf = 10;            % [s] Tempo total da simulação
dados.geral.guia = 0.70;        % [m] Tamanho da guia linear

% ==============================
% Dados do pêndulo
% ==============================
dados.pendulo.m = 0.13;        % [kg]    Massa da haste do pêndulo
dados.pendulo.l = 0.2;             % [m]     Distância do ponto de fixação até o centro de gravidade
dados.pendulo.I = 0.00007892;       % [kg·m^2] Momento de inércia da haste em relação ao centro de gravidade
                                    % Inércia em relação ao PIVÔ é J = I + m*l^2
dados.pendulo.J = dados.pendulo.I + dados.pendulo.m * dados.pendulo.l^2;
dados.pendulo.b = 0.0001;           % [N·m·s] Coeficiente de amortecimento viscoso no eixo de fixação

% ==============================
% Dados do carro
% ==============================
dados.carro.m = 0.30882;         % [kg]    Massa do carro
dados.carro.c = 1;                   % [N·s/m] Coeficiente de amortecimento viscoso entre carro e guia

% ==============================
% Dados do motor
% ==============================
dados.motor.Rm = 12.5;         % [Ω]      Resistência do enrolamento
dados.motor.Kb = 0.031;        % [V·s/rad] Constante de força contra-eletromotriz
dados.motor.Kt = 0.17;         % [N·m/A]  Constante de torque
dados.motor.r  = 0.071;        % [-]      Relação de transmissão (ou raio do pinhão)
dados.motor.V_max = 12;        % [V]      Tensão máxima de alimentação do motor (suposição)

%% Projeto do Controlador Swing-Up
% ==============================
% Ganhos do controlador
k_e = 50.0;     % Ganho de energia
k_d = 0.1;     % Ganho de amortecimento

E_alvo = dados.pendulo.m * dados.geral.g * dados.pendulo.l; % Energia no ponto mais alto

%% Configuração da Simulação
% ==============================
dt = 0.01;     % passo de tempo (s)
t = 0:dt:dados.geral.Tf; % vetor de tempo
N = length(t) - 1;

% Vetores de estado [x, x_dot, theta, theta_dot]
estado = zeros(N+1, 4);

% Vetores para vizualizar registros de controle (logs)
V_log = zeros(N, 1);
F_log = zeros(N, 1);
u_log = zeros(N, 1);

% Condições Iniciais
estado(1, :) = [0.1, 0, 0.1, 0]; % [x, x_dot, theta, theta_dot]
% Carrinho iniciando próximo do centro (à 1 mm do centro), sem volicidade e
% haste começando próximo de theta = 0 e sem velocidade angular.

%% Execução da Simulação
% ==============================
for i = 1:N
    % Ler o estado atual
    x = estado(i, 1);
    x_dot = estado(i, 2);
    theta = estado(i, 3);
    theta_dot = estado(i, 4);
    
    % --- Passo A: CÁLCULO DO CONTROLADOR ---
    
    % 1. Calcular Força Desejada (u(t))
    E_p = 0.5 * dados.pendulo.J * theta_dot^2 - dados.pendulo.m * dados.geral.g * dados.pendulo.l * cos(theta);
    E_erro = E_p - E_alvo;
    
    u_energia = k_e * E_erro * cos(theta) * theta_dot;
    
    u_desejada = u_energia - k_d * x_dot;
    
    % 2. Calcular Voltagem Real (V_real)
    V_desejada = (u_desejada * dados.motor.r * dados.motor.Rm / dados.motor.Kt) + ...
                 (dados.motor.Kb * x_dot / dados.motor.r);
                 
    V_real = max(-dados.motor.V_max, min(dados.motor.V_max, V_desejada));
    
    % 3. Calcular Força Real (F_real)
    i_real = (V_real - (dados.motor.Kb * x_dot / dados.motor.r)) / dados.motor.Rm;
    tau_real = dados.motor.Kt * i_real;
    F_real = tau_real / dados.motor.r;
    
    % Guardar logs
    V_log(i) = V_real;
    F_log(i) = F_real;
    u_log(i) = u_desejada;
    
    % CÁLCULO DA FÍSICA (A = M \ V)
    M_mat = [(dados.carro.m + dados.pendulo.m), dados.pendulo.m * dados.pendulo.l * cos(theta);
             dados.pendulo.m * dados.pendulo.l * cos(theta), dados.pendulo.J];
         
    V_vec = [F_real - dados.carro.c * x_dot + dados.pendulo.m * dados.pendulo.l * sin(theta) * theta_dot^2;
             -dados.pendulo.m * dados.geral.g * dados.pendulo.l * sin(theta) - dados.pendulo.b * theta_dot];
         
    Acel = M_mat \ V_vec;
    x_ddot = Acel(1);
    theta_ddot = Acel(2);
    
    % Atualizar o estado
    estado(i+1, 2) = x_dot + x_ddot * dt;
    estado(i+1, 4) = theta_dot + theta_ddot * dt;
    estado(i+1, 1) = x + estado(i+1, 2) * dt;
    estado(i+1, 3) = theta + estado(i+1, 4) * dt;
    estado(i+1, 3) = atan2(sin(estado(i+1, 3)), cos(estado(i+1, 3)));
end

%% 5. Visualização dos 4 Estados
% Gráficos de Estado
figure;
subplot(2,2,1); plot(t, estado(:,1)); title("Posição do Carrinho (x) [metros]"); 
xlabel('Tempo (s)'); ylabel('Posição (m)'); grid on;
subplot(2,2,2); plot(t, estado(:,2)); title("Velocidade do Carrinho (x')"); 
xlabel('Tempo (s)'); ylabel('Velocidade (m/s)'); grid on;
subplot(2,2,3); plot(t, estado(:,3) * 180/pi); title("Ângulo do Pêndulo (\theta) [graus]"); 
xlabel('Tempo (s)'); ylabel('Ângulo (graus)'); grid on;
subplot(2,2,4); plot(t, estado(:,4)); title("Velocidade Angular (\theta')"); 
xlabel('Tempo (s)'); ylabel('Velocidade (rad/s)'); grid on;

% Gráficos de Controle
figure;
plot(t(1:N), V_log, 'r', 'LineWidth', 1.5); hold on;
plot(t(1:N), F_log, 'b', 'LineWidth', 1.5);
plot(t(1:N), u_log, 'g--');
title('Logs de Controle'); 
legend('V_{real} (V)', 'F_{real} (N)', 'u_{desejada} (N)', 'Location', 'SouthEast');
xlabel('Tempo (s)'); grid on;
ylim([-15 15]);