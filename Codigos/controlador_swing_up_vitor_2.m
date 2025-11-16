%% Limpeza do Workspace
clear; clc; close all;

%% ==============================
% Carregar Dados do Projeto
% ==============================
dados.geral.g = 9.81;
dados.geral.Tf = 20;
dados.pendulo.m = 0.13;
dados.pendulo.l = 0.2;
dados.pendulo.I = 0.00007892;
dados.pendulo.J = dados.pendulo.I + dados.pendulo.m * dados.pendulo.l^2;
dados.pendulo.b = 0.0001;
dados.carro.m = 0.30882;
dados.carro.c = 1;

%% ==============================
% Script Principal - Simulação "Caminho 2" (Modelo Completo)
% ==============================

% --- 1. Definir Parâmetros da Simulação E CONTROLE ---
Tf = dados.geral.Tf; % Tempo de simulação
k = 6.0; % Ganho de Energia (k)

% NOVOS GANHOS (Anti-Drift):
k_p = 1.0;  % Ganho "Mola" (Proporcional)
k_d = 2.0;  % Ganho "Amortecedor" (Derivativo)
% (Estes são "chutes" iniciais, talvez precisemos ajustá-los)

tspan = [0 Tf];

% --- 2. Definir Condições Iniciais (4 estados) ---
% Convenção: 0 = pêndulo para baixo
x0 = [0;   % x (posição inicial do carro)
      0;   % x_dot (velocidade inicial do carro)
      0;   % theta (ângulo inicial, 0 = para baixo)
      0];  % theta_dot (velocidade angular inicial)

% --- 3. Chamar o Solver (ode45) ---
% Passamos 'k', 'k_p', 'k_d' e 'dados' para a nossa função
[t, x_vec] = ode45(@(t_arg, x_arg) cart_pendulum_dynamics(t_arg, x_arg, k, k_p, k_d, dados), tspan, x0);

% --- 4. Exibir Confirmação ---
disp('Simulação CAMINHO 2 concluída!');
disp(['Simulado com k = ' num2str(k) ', k_p = ' num2str(k_p) ', k_d = ' num2str(k_d)]);

%% ==============================
% Pós-Processamento e Plots (Modelo Completo)
% ==============================

% --- 5. Extrair Estados ---
x_pos = x_vec(:, 1);
x_vel = x_vec(:, 2);
th = x_vec(:, 3);
th_dot = x_vec(:, 4);

% --- 6. Recalcular Energia e Força para cada Ponto ---
n_passos = length(t);
E = zeros(n_passos, 1);
F = zeros(n_passos, 1);

% Constantes (para facilitar)
m_c = dados.carro.m; m_b = dados.pendulo.m; L_b = dados.pendulo.l;
g = dados.geral.g; c = dados.carro.c; b = dados.pendulo.b;
J_piv = dados.pendulo.J;

for i = 1:n_passos
    % Pega o estado atual do loop
    xt = x_pos(i); x_dott = x_vel(i); tht = th(i); th_dott = th_dot(i);
    
    % Calcular Energia (Eq. do Loop Externo)
    E(i) = 0.5 * J_piv * th_dott^2 - m_b * g * L_b * (cos(tht) + 1);
    
    % Recalcular Força (Eq. do Loop Interno)
    
    % (Repetir a lógica da função ode)
    arg = th_dott * cos(tht);
    if arg == 0, sign_arg = 1.0; else, sign_arg = sign(arg); end
    
    % Lei de controle combinada
    termo_energia = k * E(i) * sign_arg;
    termo_posicao = -k_p * xt - k_d * x_dott; % Adiciona o feedback de posição
    x_ddot_desejado = termo_energia + termo_posicao; % Combina os objetivos

    th_ddot_result = (-b*th_dott - m_b*g*L_b*sin(tht) - m_b*L_b*cos(tht)*x_ddot_desejado) / (m_b*L_b^2);
    F(i) = (m_c + m_b)*x_ddot_desejado + m_b*L_b*cos(tht)*th_ddot_result + c*x_dott - m_b*L_b*sin(tht)*th_dott^2;
end

%% --- 7. Plotar os Resultados (6 gráficos) ---
figure;

% Posição do Carro
subplot(3, 2, 1);
plot(t, x_pos, 'b-');
title(['Swing-Up, k=' num2str(k) ', kp=' num2str(k_p) ', kd=' num2str(k_d)]);
ylabel('Pos. Carro (m)');
grid on;

% Velocidade do Carro
subplot(3, 2, 2);
plot(t, x_vel, 'b-');
ylabel('Vel. Carro (m/s)');
grid on;

% Ângulo do Pêndulo (em Graus)
subplot(3, 2, 3);
plot(t, th * (180/pi), 'r-');
ylabel('Ângulo Pênd. (graus)');
grid on;

% Velocidade Angular do Pêndulo
subplot(3, 2, 4);
plot(t, th_dot, 'r-');
ylabel('Vel. Ang. (rad/s)');
grid on;

% Energia do Pêndulo
subplot(3, 2, 5);
plot(t, E, 'k-');
ylabel('Energia (J)');
xlabel('Tempo (s)');
grid on;

% Força de Controle (F)
subplot(3, 2, 6);
plot(t, F, 'g-');
ylabel('Força (N)');
xlabel('Tempo (s)');
grid on;

disp('Gráficos gerados!');

function dxdt = cart_pendulum_dynamics(t, x_vec, k, k_p, k_d, dados)
    % Esta função simula o sistema Carro-Pêndulo completo.

    % --- 1. Definir Constantes Físicas (dos seus dados) ---
    m_c = dados.carro.m;        % Massa do carrinho
    m_b = dados.pendulo.m;      % Massa do pêndulo
    L_b = dados.pendulo.l;      % Comprimento ao CM do pêndulo
    g = dados.geral.g;
    c = dados.carro.c;          % Atrito do carrinho
    b = dados.pendulo.b;        % Atrito do pêndulo
    J_piv = dados.pendulo.J;    % Inércia do pêndulo no pivô (para energia)
    
    % --- 2. Desempacotar o Vetor de Estado (Convenção: 0=baixo) ---
    % x_vec(1) = x     (posição do carro)
    % x_vec(2) = x_dot (velocidade do carro)
    % x_vec(3) = th    (ângulo do pêndulo)
    % x_vec(4) = th_dot (velocidade angular do pêndulo)
    x_dot = x_vec(2);
    th = x_vec(3);
    th_dot = x_vec(4);

    % --- 3. Loop Externo (Estratégia de Energia + Posição) ---
    % Convenção: 0=baixo, pi=cima
    E = 0.5 * J_piv * th_dot^2 - m_b * g * L_b * (cos(th) + 1);
    E0 = 0; % Energia alvo no topo (th = pi)
        
    % Argumento da função sign
    arg = th_dot * cos(th); 
    
    % Correção para o 'sign(0)' (baseado na Seção 4 do artigo)
    if arg == 0
        sign_arg = 1.0; % Dá o "chute" inicial
    else
        sign_arg = sign(arg);
    end
    
    % --- NOVA LEI DE CONTROLE COM MÚLTIPLOS OBJETIVOS ---
    % Objetivo 1: Termo de Energia (Swing-Up)
    termo_energia = k * E * sign_arg;
    
    % Pega os estados do carrinho para o Objetivo 2
    x_pos = x_vec(1);
    x_vel = x_vec(2);
    
    % Objetivo 2: Termo de Posição (Anti-Drift)
    termo_posicao = -k_p * x_pos - k_d * x_vel;
    
    % Lei de Controle Combinada
    x_ddot_desejado = termo_energia + termo_posicao;


    % --- 4. Loop Interno (Cálculo da Força 'F') ---
    % (Usando Dinâmica Inversa para encontrar a força 'F' necessária)
    
    % 4a. Calcular o theta_ddot resultante
    th_ddot_result = (-b*th_dot - m_b*g*L_b*sin(th) - m_b*L_b*cos(th)*x_ddot_desejado) / (m_b*L_b^2);
    
    % 4b. Calcular a Força F necessária para atingir ambos
    F = (m_c + m_b)*x_ddot_desejado + m_b*L_b*cos(th)*th_ddot_result + c*x_dot - m_b*L_b*sin(th)*th_dot^2;

    % --- 5. Simulação da Física Real (O que a ode45 resolve) ---
    % Dadas as condições atuais e a Força F, quais são as acelerações REAIS?
    
    % Montar a Matriz de Massa M
    M = [(m_c + m_b), m_b*L_b*cos(th);
         m_b*L_b*cos(th), (m_b*L_b^2)];
         
    % Montar o Vetor V (com a Força F que calculamos)
    V = [F - c*x_dot + m_b*L_b*sin(th)*th_dot^2;
         -b*th_dot - m_b*g*L_b*sin(th)];
         
    % Resolver para as acelerações reais
    aceleracoes_reais = M \ V;
    
    x_ddot_real = aceleracoes_reais(1);
    th_ddot_real = aceleracoes_reais(2);
    
    % --- 6. Retornar o Vetor de Derivadas ---
    dxdt = zeros(4,1);
    dxdt(1) = x_vec(2);       % dx/dt = x_dot
    dxdt(2) = x_ddot_real;    % dx_dot/dt = x_ddot_real
    dxdt(3) = x_vec(4);       % dth/dt = th_dot
    dxdt(4) = th_ddot_real;   % dth_dot/dt = th_ddot_real
    
end