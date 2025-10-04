%% Modelo do sistema do pêndulo invertido

clear; clc; close all;

%% Declaração das variáveis do sistema

% % ==============================
% % Dados gerais
% % ==============================
% dados.geral.g = 9.81;          % [m/s^2] Aceleração da gravidade
% dados.geral.Ts = 10/1000;       % [s] Período de amostragem
% dados.geral.Tf = 10;            % [s] Tempo total da simulação
% dados.geral.spt = 10;         % [cm] Posição desejada do carro
% dados.geral.angulo_troca = 30;         % [°] Ângulo para troca dos controladores
% dados.geral.guia = 70; % [cm] Tamanho da guia linear
% 
% % Condições Iniciais (pêndulo para baixo)
% dados.geral.inicial.theta0 = 160;      % [°] Ângulo inicial
% dados.geral.inicial.theta_dot0 = 0;   % [°/s] Velocidade angular inicial
% dados.geral.inicial.x0 = 20/100;           % [m] Posição inicial do carro
% dados.geral.inicial.x_dot0 = 0;       % [m/s] Velocidade inicial do carro
% 
% % ==============================
% % Dados do pêndulo
% % ==============================
% dados.pendulo.m = 0.1;        % [kg]    Massa da haste do pêndulo
% dados.pendulo.l = 0.2;        % [m]     Distância do ponto de fixação até o centro de gravidade
% dados.pendulo.I = 0.0007176;  % [kg·m^2] Momento de inércia da haste em relação ao centro
% dados.pendulo.b = 0.00007892;   % [N·m·s] Coeficiente de amortecimento viscoso no eixo de fixação
% dados.pendulo.r_massa = 0.01; % [m] Raio da massa na ponta do pêndulo 
% dados.pendulo.E_des = 2*dados.pendulo.m*dados.geral.g*dados.pendulo.l; % [J] Energia Potencial Desejada do Pêndulo
% 
% % ==============================
% % Dados do carro
% % ==============================
% dados.carro.m = 0.135;         % [kg]    Massa do carro
% dados.carro.c = 0.63;          % [N·s/m] Coeficiente de amortecimento viscoso entre carro e guia
% dados.carro.l = 0.1;           % [m] Largura do carro
% dados.carro.h = 0.05;           % [m] Altura do carro
% 
% % ==============================
% % Dados do motor
% % ==============================
% dados.motor.J  = 2.6*10^(-4);         % [kg·m^2] Inércia do rotor (Chute)
% dados.motor.Rm = 12.5;         % [Ω]      Resistência do enrolamento (Chute)
% dados.motor.Kb = 0.031;         % [V·s/rad] Constante de força contra-eletromotriz (Chute)
% dados.motor.Kt = 0.031;         % [N·m/A]  Constante de torque (Torque/Corrente)
% dados.motor.R  = 0.006;         % [-]      Relação de transmissão (1/14)

% ==============================
% Dados gerais
% ==============================
dados.geral.g = 9.81;          % [m/s^2] Aceleração da gravidade
dados.geral.Ts = 10/1000;       % [s] Período de amostragem
dados.geral.Tf = 10;            % [s] Tempo total da simulação
dados.geral.spt = 0;         % [cm] Posição desejada do carro
dados.geral.angulo_troca = 30;         % [°] Ângulo para troca dos controladores
dados.geral.guia = 70; % [cm] Tamanho da guia linear

% Condições Iniciais (pêndulo para baixo)
dados.geral.inicial.theta0 = 170;      % [°] Ângulo inicial
dados.geral.inicial.theta_dot0 = 0;   % [°/s] Velocidade angular inicial
dados.geral.inicial.x0 = 0;           % [m] Posição inicial do carro
dados.geral.inicial.x_dot0 = 0;       % [m/s] Velocidade inicial do carro

% ==============================
% Dados do pêndulo
% ==============================
dados.pendulo.m = 55.5/1000;        % [kg]    Massa da haste do pêndulo
dados.pendulo.l = 0.45/2;        % [m]     Distância do ponto de fixação até o centro de gravidade
dados.pendulo.I = (1/12)*dados.pendulo.m*(dados.pendulo.l*2)^2;  % [kg·m^2] Momento de inércia da haste em relação ao centro
dados.pendulo.b = 0.00007892;   % [N·m·s] Coeficiente de amortecimento viscoso no eixo de fixação
dados.pendulo.r_massa = 0.01; % [m] Raio da massa na ponta do pêndulo 
dados.pendulo.E_des = 2*dados.pendulo.m*dados.geral.g*dados.pendulo.l; % [J] Energia Potencial Desejada do Pêndulo

% ==============================
% Dados do carro
% ==============================
dados.carro.m = 308.82/1000;         % [kg]    Massa do carro
dados.carro.c = 0.002;          % [N·s/m] Coeficiente de amortecimento viscoso entre carro e guia
dados.carro.l = 0.1;           % [m] Largura do carro
dados.carro.h = 0.05;           % [m] Altura do carro

% ==============================
% Dados do motor
% ==============================
dados.motor.Rm = 12.5;         % [Ω]      Resistência do enrolamento (Chute)
dados.motor.Kb = 0.031;         % [V·s/rad] Constante de força contra-eletromotriz (Chute)
dados.motor.Kt = 0.17;         % [N·m/A]  Constante de torque (Torque/Corrente)
dados.motor.R  = 0.071;         % [-]      Relação de transmissão (1/14)

%% Geração das matrizes de espaço de estados

% Estados x1=x, x2=theta, x3=x_ponto, x4=theta_ponto
g = dados.geral.g;
m = dados.pendulo.m;
l = dados.pendulo.l;
I = dados.pendulo.I;
b = dados.pendulo.b;
M = dados.carro.m;
c = dados.carro.c;
Rm = dados.motor.Rm;
Kb = dados.motor.Kb;
Kt = dados.motor.Kt;
r = dados.motor.R;
tau = dados.geral.Ts;

alpha = (I + m*l^2)*(M + m) - (m*l)^2;

% Matrizes de estado para utilizar o sinal de comando Tensão [V]
Ac = [0, 0, 1, 0;
     0, 0, 0, 1;
     0, (m^2*l^2*g)/alpha, -(I + m*l^2)*(c + (Kt*Kb)/(Rm*r^2))/alpha, -b*m*l/alpha;
     0, m*g*l*(M + m)/alpha, -m*l*(c + (Kt*Kb)/(Rm*r^2))/alpha, -b*(M + m)/alpha];

Bc = [0;
     0;
    (I + m*l^2)*Kt/(alpha*Rm*r);
     m*l*Kt/(alpha*Rm*r)];

% Matrizes de estado para utilizar o sinal de comando Força [N]
% A = [0, 0, 1, 0;
%      0, 0, 0, 1;
%      0, (m^2*l^2*g)/alpha, -(I + m*l^2)*c/alpha, -b*m*l/alpha;
%      0, m*g*l*(M + m)/alpha, -m*l*c/alpha, -b*(M + m)/alpha];
% 
% B = [0;
%      0;
%     (I + m*l^2)/alpha;
%      m*l/alpha];

C = eye(4);
D = 0;

planta.Ac = Ac;
planta.Bc = Bc;
planta.C = C;
planta.D = D;

[A, B] = c2d(Ac,Bc,tau);

planta.A = A;
planta.B = B;

clear g m l I b M c Rm Kb Kt r alpha A B C D Ac Bc;

%% Estabilidade do sistema

Poles = eig(planta.A);
fprintf("O sistema possui os seguintes polos\n")
disp(Poles);
fprintf("Nota-se que um dos polos está no semi plano direito, de modo que o sistema é instável");

clear Poles;

%% Controlabilidade do sistema

matriz_controlabilidade = ctrb(planta.A, planta.B);
Rank = rank(matriz_controlabilidade);
fprintf("O posto da matriz é: ");
disp(Rank);
fprintf("ou seja, é possível controlar todos os estados do sistema.")

clear matriz_controlabilidade Rank;

%% Observabilidade do sistema

matriz_observabilidade = obsv(planta.A, planta.C);
Rank = rank(matriz_observabilidade);
fprintf("\nO posto da matriz é: ");
disp(Rank);
fprintf("ou seja, é possível observar todos os estados do sistema.")

clear matriz_observabilidade Rank;

%% Simulação do sistema em malha aberta 

% t = 0:0.1:5;
% [y,x,t] = step(planta.A,planta.B,planta.C,planta.D,1,t);
% 
% subplot(2,2,1);
% plot(t,x(:,1));
% subplot(2,2,2);
% plot(t,x(:,2));
% subplot(2,2,3);
% plot(t,x(:,3));
% subplot(2,2,4);
% plot(t,x(:,4));