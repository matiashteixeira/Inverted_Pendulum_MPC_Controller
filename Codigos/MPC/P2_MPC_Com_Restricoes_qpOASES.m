%% Controlador Preditivo Com Restrições

run Codigos\P1_Modelo_Do_Sistema.m;
close all;

%% Variáveis gerais

A = dados.planta.A;
B = dados.planta.B; 
tau = dados.geral.Ts; 

%pos_limite = dados.geral.guia/2*0.8;
pos_limite = inf/100;
ang_limite = inf*(pi/180);
vel_limite = inf/100;
vel_ang_limite = inf;
comando_limite = inf;

ang_inicial = 185*(pi/180);
pos_inicial = 0/100;

pos_spt = 0/100;

tsim=10;

qp_error_count = 0;
QP_MAX_FAIL = 5;

%% Variáveis do MPC

[~,nu] = size(B); 

MPC.A = A; 
MPC.B = B;

% Definição das variáveis rastreadas
MPC.Cr = eye(4);

MPC.Qy = diag([250 100 1 1]);
MPC.Qu = 0.01;
MPC.N = 50; 

% Definição das variáveis restringidas
MPC.Cc = eye(4);

% Definição das restrições
MPC.ycmin = [-pos_limite; -ang_limite; -vel_limite; -vel_ang_limite]; MPC.ycmax= [pos_limite; ang_limite; vel_limite; vel_ang_limite]; 
MPC.umin = -comando_limite; MPC.umax = comando_limite; 
MPC.ulast = 0; MPC.deltamin=-100000; MPC.deltamax=100000;

% Cálculo das matrizes do MPC
[MPC]=compute_MPC_Matrices(MPC);

% Condições Iniciais
x0=[pos_inicial;ang_inicial;0;0];
u = 0;

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
yref(1:4:end) = pos_spt;  
yref(2:4:end) = pi;  

lesx(1,:)=x0';
lesy(1,:) = MPC.Cr*lesx(1, :)'; 

x_des = [0 180*pi/180 0 0];

% Opções para o qpOASES e inicialização
options_qpOASES = qpOASES_options('maxIter', 100000);
QP = [];

sat = @(x, x_max, x_min) min( x_max, max(x_min,x));

for i=1:nt-MPC.N

    xplus = RK4_discrete(lesx(i,:),u,tau,dados);


    yref_pred=yref(i*num_var_reguladas + 1 : (i+MPC.N)*num_var_reguladas);

    %if abs(i - (nt-1)/2) < tau/2
    %    lesx(i,4) = lesx(i,4) + 80*pi/180;
    %end
    err = xplus-x_des;

    F=MPC.F1*err'+MPC.F2*yref_pred;
    Bineq=MPC.G1*xplus'+MPC.G2*MPC.ulast+MPC.G3; 

    try
        if i == 1 || isempty(QP)
            [QP, utilde_opt, ~, exitflag, ~, ~] = qpOASES_sequence('i', MPC.H, F, MPC.Aineq, MPC.utildemin, MPC.utildemax, [], Bineq, options_qpOASES);
        else
            [utilde_opt, ~, exitflag, ~, ~] = qpOASES_sequence('h', QP, F, MPC.utildemin, MPC.utildemax, [], Bineq, options_qpOASES);
        end

        % Se não convergiu → conta erro
        if exitflag ~= 0 || any(~isfinite(utilde_opt))
            qp_error_count = qp_error_count + 1;
        else
            qp_error_count = 0;
        end
    
    catch
        qp_error_count = qp_error_count + 1;
    end
    
    %if qp_error_count >= QP_MAX_FAIL
        % SwingUp
    %    u = EnergySwingUpController(lesx(i,:)', dados);
    %    u = sat(u,12,-12);
    %else
        % MPC
        u=P_i(1, nu, MPC.N)*utilde_opt;
    %end
    
    lesu (i, :)=u'; 
    MPC.ulast=u;
    lesx(i+1, :)=xplus;
    lesy(i+1, :)=MPC.Cr*xplus';

end

Nplot = nt - MPC.N;

posicao = lesx(1:Nplot,1).*100;
angulo = lesx(1:Nplot,2).*180/pi;
velocidade = lesx(1:Nplot,3).*100;
vel_angular = lesx(1:Nplot,4).*180/pi;
comando = lesu(1:Nplot);
tempo = lest(1:Nplot);

pos_ref = yref(1:4:4*Nplot) * 100;      
ang_ref = yref(2:4:4*Nplot) * 180/pi;   
vel_ref = yref(3:4:4*Nplot) * 100;      
vag_ref = yref(4:4:4*Nplot) * 180/pi; 

clear A B Bineq err F i lest lesy lesx lesu MPC n nt nu num_var_reguladas Nplot options_qpOASES pos_inicial;
clear ang_inicial pos_spt QP tau tsim u utilde_opt x0 x_des xplus yref yref_pred;

%% Plot dos resultados

% ================== COMANDO (u) ==================

figure;
hold on; grid on;

% Comando (linha contínua grossa e colorida)
stairs(tempo, comando, 'LineWidth', 2.5, 'Color', [0 0.45 0.74]);  % azul

% Limites (pontilhado vermelho mais grosso)
plot(tempo, comando_limite * ones(size(tempo)), 'r--', 'LineWidth', 2.5);
plot(tempo, -comando_limite * ones(size(tempo)), 'r--', 'LineWidth', 2.5);

title('Comando (V)');
xlabel('Tempo (s)');
ylabel('Tensão (V)');
legend('Comando','Limite superior','Limite inferior','Location','best');

% ================== POSIÇÃO ==================
figure;
hold on; grid on;

plot(tempo, posicao, 'LineWidth', 2.5, 'Color', [0 0.45 0.74])
plot(tempo, pos_ref,  '--', 'LineWidth', 2.5, 'Color', [0.93 0.69 0.13]) % referência (amarelo)

plot(tempo,  pos_limite*100 * ones(size(tempo)),  'r--', 'LineWidth', 2.5)
plot(tempo, -pos_limite*100 * ones(size(tempo)),  'r--', 'LineWidth', 2.5)

title('Posição (cm)')
xlabel('Tempo (s)')
ylabel('Posição (cm)')
legend('Posição','Referência','Limite Sup','Limite Inf','Location','best')



% ================== ÂNGULO ==================
figure;
hold on; grid on;

plot(tempo, angulo, 'LineWidth', 2.5, 'Color', [0 0.45 0.74])
plot(tempo, ang_ref, '--', 'LineWidth', 2.5, 'Color', [0.93 0.69 0.13])

plot(tempo,  180+ang_limite*180/pi * ones(size(tempo)),  'r--', 'LineWidth', 2.5)
plot(tempo, 180-ang_limite*180/pi * ones(size(tempo)),  'r--', 'LineWidth', 2.5)

title('Ângulo (°)')
xlabel('Tempo (s)')
ylabel('Ângulo (°)')
legend('Ângulo','Referência','Limite Sup','Limite Inf','Location','best')



% ================== VELOCIDADE ==================
figure;
hold on; grid on;

plot(tempo, velocidade, 'LineWidth', 2.5, 'Color', [0 0.45 0.74])
plot(tempo, vel_ref,    '--', 'LineWidth', 2.5, 'Color', [0.93 0.69 0.13])

plot(tempo,  vel_limite*100 * ones(size(tempo)),  'r--', 'LineWidth', 2.5)
plot(tempo, -vel_limite*100 * ones(size(tempo)),  'r--', 'LineWidth', 2.5)

title('Velocidade (cm/s)')
xlabel('Tempo (s)')
ylabel('Velocidade (cm/s)')
legend('Velocidade','Referência','Limite Sup','Limite Inf','Location','best')



% ================== VELOCIDADE ANGULAR ==================
figure;
hold on; grid on;

plot(tempo, vel_angular, 'LineWidth', 2.5, 'Color', [0 0.45 0.74])
plot(tempo, vag_ref,     '--', 'LineWidth', 2.5, 'Color', [0.93 0.69 0.13])

plot(tempo,  vel_ang_limite * ones(size(tempo)),  'r--', 'LineWidth', 2.5)
plot(tempo, -vel_ang_limite * ones(size(tempo)),  'r--', 'LineWidth', 2.5)

title('Velocidade Angular (°/s)')
xlabel('Tempo (s)')
ylabel('Vel Angular (°/s)')
legend('Vel. Angular','Referência','Limite Sup','Limite Inf','Location','best')

clear ang_limite ang_ref angulo pos_limite comando comando_limite pos_ref posicao tempo vag_ref vel_ang_limite vel_angular vel_limite vel_ref velocidade;
%%

% print_cpp_matrix("H", MPC.H);
% print_cpp_matrix("F1", MPC.F1);
% print_cpp_matrix("F2", MPC.F2);
% print_cpp_matrix("Aineq", MPC.Aineq);
% print_cpp_matrix("G1", MPC.G1);
% print_cpp_matrix("G2", MPC.G2);
% print_cpp_matrix("G3", MPC.G3);
% print_cpp_matrix("utildemin", MPC.utildemin);
% print_cpp_matrix("utildemax", MPC.utildemax);