%% Controlador Preditivo Com Restrições

run Codigos\P1_Modelo_Do_Sistema.m;

close all;

%% Variáveis gerais

A = dados.planta.A;
B = dados.planta.B; 
tau = dados.geral.Ts; 

%pos_limite = dados.geral.guia/2*0.8;
pos_limite = 20/100;
ang_limite = 5*(pi/180);
comando_limite = 12;

ang_inicial = 5*(pi/180);
pos_inicial = 20/100;

pos_spt = 0/100;

tsim=30;

%% Variáveis do MPC

[n,nu] = size(B); 

MPC.A = A; 
MPC.B = B;

% Definição das variáveis rastreadas
MPC.Cr = [1 0 0 0; 0 1 0 0]; 

MPC.Qy = [500 0; 0 1000];
MPC.Qu = 0.001; 
MPC.N = 40; 

% Definição das variáveis restringidas
MPC.Cc = [1 0 0 0; 0 1 0 0];

% Definição das restrições
MPC.ycmin = [-pos_limite; -ang_limite]; MPC.ycmax= [pos_limite; ang_limite]; 
MPC.umin = -comando_limite; MPC.umax = comando_limite; 
MPC.ulast = 0; MPC.deltamin=-inf; MPC.deltamax=inf;

% Cálculo das matrizes do MPC
[MPC]=compute_MPC_Matrices(MPC);

% Condições Iniciais
x0=[pos_inicial;ang_inicial;0;0];

% Inicialização do lest e captura do tamanho dos vetores 
lest=(0:tau:tsim)';
nt=size(lest, 1);
num_var_reguladas = size(MPC.Cr,1);

% Inicialização dos vetores
lesx=zeros(nt,length(B)); 
lesy=zeros(nt, num_var_reguladas); 
lesu=zeros(nt, 1);

% Sinal de referência
yref = zeros(length(lest)*num_var_reguladas,1);         
yref(1:2:end) = pos_spt;  

lesx(1,:)=x0';
lesy(1,:) = MPC.Cr*lesx(1, :)'; 

% Opções para o qpOASES e inicialização
options_qpOASES = qpOASES_options('maxIter', 100000);
QP = [];

for i=1:nt-1
    yref_pred=yref(i+1:i+MPC.N*num_var_reguladas);


    % Aplicação de um distúrbio aos 20s
    if abs(i - (nt-1)/2) < tau/2
        lesx(i,4) = lesx(i,4) + 75*pi/180;
    end

    F=MPC.F1*lesx(i,:)'+MPC.F2*yref_pred;
    Bineq=MPC.G1*lesx(i, :)'+MPC.G2*MPC.ulast+MPC.G3; 


    if i == 1
        % Passo 1: Inicialização e Cold Start ('i')
        [QP, utilde_opt, ~, ~, ~] = qpOASES_sequence('i', MPC.H, F, MPC.Aineq, MPC.utildemin, MPC.utildemax, [], Bineq, options_qpOASES);
    else
        % Passo 2: Hot Start ('h') - reutiliza o QP e atualiza os parâmetros
        [utilde_opt, ~, ~, ~] = qpOASES_sequence('h', QP, F, MPC.utildemin, MPC.utildemax, [], Bineq, options_qpOASES);
    end


    u=P_i(1, nu, MPC.N)*utilde_opt;
    lesu (i, :)=u'; 
    MPC.ulast=u;
    xplus=MPC.A*lesx(i, :)'+MPC.B*u;
    lesx(i+1, :)=xplus';
    lesy(i+1, :)=MPC.Cr*xplus;

end

%% Plot dos resultados

subplot(2,2,1);
stairs(lest,lesu); grid on;
title('Comando (v)');
xlabel("Tempo (s)");

pos_simulado = lesy(:,1).*100;

limitacao_superior = ones(nt, 1) * pos_limite;
limitacao_inferior = ones(nt, 1) * -pos_limite;

subplot(2,2,2);
plot(lest,pos_simulado,lest,yref(1:2:end).*100,lest,limitacao_superior.*100,'k',lest,limitacao_inferior.*100,'k'); grid on;
title('Posição (cm)');
xlabel("Tempo (s)");
legend('Posição Simulada', 'Referência', 'Location', 'best');
ylim([-25 25]);


% ang_simulado = lesy(:,2).*(180/pi);
% 
% subplot(2,2,3);
% plot(lest,ang_simulado,lest,yref(2:2:end)); grid on;
% title('Ângulo (°)');

ang_simulado_desvio = lesy(:,2).*(180/pi); % Desvio em graus (Delta_theta)
ang_simulado_abs = ang_simulado_desvio + 180; % Angulo Absoluto (theta_abs)

yref_ang_abs = ones(nt, 1) * 180;

limitacao_superior = ones(nt, 1) * ang_limite.*(180/pi) + 180;
limitacao_inferior = ones(nt, 1) * -ang_limite.*(180/pi) + 180;

subplot(2,2,3);
plot(lest, ang_simulado_abs, lest, yref_ang_abs, lest, limitacao_superior, 'k', lest, limitacao_inferior, 'k');
xlabel("Tempo (s)");
grid on;
title('Ângulo (°)');
legend('Ângulo Simulado (Absoluto)', 'Referência (180°)', 'Location', 'best');
ylim([170 187]);

