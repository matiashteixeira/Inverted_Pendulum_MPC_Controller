%% Limpeza do Workspace

clear; 
clc; 
close all;

%% Dados

dados.geral.g = 9.81;           % [m/s^2] Gravidade
dados.pendulo.m = 0.13;         % [kg] Massa
dados.pendulo.l = 0.2;          % [m] Distância ao centro de massa
dados.pendulo.I = 0.00007892;   % [kg·m^2] Inércia no CM
dados.pendulo.b = 0.0001;       % [N·m·s] Atrito viscoso no pivô
dados.pendulo.J = dados.pendulo.I + dados.pendulo.m * dados.pendulo.l^2;
dados.carro.m = 0.30882;        % [kg] Massa
dados.carro.c = 1;              % [N·s/m] Atrito viscoso do carrinho

%% Parâmetros do Controlador

k = 12.0;   % Ganho do controlador de Energia (swing-up)
k_p = 0;  % Ganho Proporcional do controle de Posição (anti-drift)
k_d = 0;  % Ganho Derivativo do controle de Posição (anti-drift)

%% Simulação

Tf = 10;                % [s] Tempo total de simulação
dt = 0.01;              % [s] Passo de tempo (Ex: 10ms, ou 100 Hz)
N = round(Tf / dt);     % Número total de passos
t = linspace(0, Tf, N); % Vetor de tempo

% Pré-alocar arrays para guardar o "histórico" dos estados
% (Isso é apenas para plotar no final)
x_hist   = zeros(N, 1);
x_dot_hist = zeros(N, 1);
th_hist    = zeros(N, 1);
th_dot_hist = zeros(N, 1);
F_hist     = zeros(N, 1);
E_hist     = zeros(N, 1);

% O sistema começa parado, com o pêndulo para baixo (theta = 0).
x_hist(1) = 0;
x_dot_hist(1) = 0;
th_hist(1) = 0;
th_dot_hist(1) = 0;
F_hist(1) = 0;
E_hist(1) = -dados.pendulo.m * dados.geral.g * dados.pendulo.l * 2; % Energia inicial

disp('Iniciando simulação de tempo discreto...');

for i = 1:(N-1)
    x_pos   = x_hist(i);
    x_vel   = x_dot_hist(i);
    th      = th_hist(i);
    th_dot  = th_dot_hist(i);
    m_c = dados.carro.m;
    m_b = dados.pendulo.m;
    L_b = dados.pendulo.l;
    g = dados.geral.g;
    c = dados.carro.c;
    b = dados.pendulo.b;
    J_piv = dados.pendulo.J;
    % 1. Calcular Energia (Loop Externo)
    E = 0.5 * J_piv * th_dot^2 - m_b * g * L_b * (cos(th) + 1);
        
    % 2. Lógica do 'sign' para o "chute inicial"
    arg = th_dot * cos(th); 
    if arg == 0
        sign_arg = 1.0; % Força o primeiro movimento
    else
        sign_arg = sign(arg);
    end
    
    % 3. Lei de Controle Multi-Objetivo (Energia + Posição)
    termo_energia = k * E * sign_arg;            % Objetivo 1: Swing-up
    termo_posicao = -k_p * x_pos - k_d * x_vel;  % Objetivo 2: Anti-drift
    
    x_ddot_desejado = termo_energia + termo_posicao; % Combinação

    % 4. Dinâmica Inversa (Calcular a Força 'F' necessária)
    th_ddot_result = (-b*th_dot - m_b*g*L_b*sin(th) - m_b*L_b*cos(th)*x_ddot_desejado) / (m_b*L_b^2);
    F = (m_c + m_b)*x_ddot_desejado + m_b*L_b*cos(th)*th_ddot_result + c*x_vel - m_b*L_b*sin(th)*th_dot^2;

    % --- "Atuar no Motor" ---
    % No ESP, aqui você converteria 'F' em uma Tensão ou PWM:
    % V_motor = F_para_Volts(F);
    % motor.write(V_motor);

    % =================================================================
    
    % Esta seção simula o que a "Física" (o mundo real) faz
    % em resposta à Força F que acabamos de calcular.
    
    % 1. Calcular Acelerações Reais (Dinâmica Direta)
    % Resolve M * [x_ddot; th_ddot] = V para as acelerações
    
    M = [(m_c + m_b), m_b*L_b*cos(th);
         m_b*L_b*cos(th), (m_b*L_b^2)];
         
    V = [F - c*x_vel + m_b*L_b*sin(th)*th_dot^2;
         -b*th_dot - m_b*g*L_b*sin(th)];
         
    aceleracoes_reais = M \ V;
    
    x_ddot_real = aceleracoes_reais(1);
    th_ddot_real = aceleracoes_reais(2);
    
    % 2. Integrar para encontrar o *próximo* estado
    
    % new_pos = old_pos + (old_vel * dt)
    x_hist(i+1) = x_pos + x_vel * dt;
    th_hist(i+1) = th + th_dot * dt;
    
    % new_vel = old_vel + (old_accel * dt)
    x_dot_hist(i+1) = x_vel + x_ddot_real * dt;
    th_dot_hist(i+1) = th_dot + th_ddot_real * dt;
    
    % Salvar valores para plotagem
    F_hist(i+1) = F;
    E_hist(i+1) = E;    
end
% Fim do loop principal

disp('Simulação concluída.');

%% 6. Plotar os Resultados
figure('Name', 'Simulação Discreta (Estilo ESP)');

% Gráfico 1: Posição do Carro (x)
subplot(3, 2, 1);
plot(t, x_hist, 'b-');
title(['Swing-Up Discreto, k=' num2str(k) ', kp=' num2str(k_p) ', kd=' num2str(k_d)]);
ylabel('Pos. Carro (m)');
grid on;

% Gráfico 2: Velocidade do Carro (x_dot)
subplot(3, 2, 2);
plot(t, x_dot_hist, 'b-');
ylabel('Vel. Carro (m/s)');
grid on;

% Gráfico 3: Ângulo do Pêndulo (theta)
subplot(3, 2, 3);
plot(t, th_hist * (180/pi), 'r-'); % Convertido para graus
ylabel('Ângulo Pênd. (graus)');
grid on;

% Gráfico 4: Velocidade Angular do Pêndulo (theta_dot)
subplot(3, 2, 4);
plot(t, th_dot_hist, 'r-');
ylabel('Vel. Ang. (rad/s)');
grid on;

% Gráfico 5: Energia do Pêndulo (E)
subplot(3, 2, 5);
plot(t, E_hist, 'k-');
ylabel('Energia (J)');
xlabel('Tempo (s)');
grid on;

% Gráfico 6: Força de Controle (F)
subplot(3, 2, 6);
plot(t, F_hist, 'g-');
ylabel('Força (N)');
xlabel('Tempo (s)');
grid on;

disp('Gráficos gerados!');

% Importem apenas com os primeiros 4 segundos, após isso o LGR entraria em
% ação tomando o controle :)