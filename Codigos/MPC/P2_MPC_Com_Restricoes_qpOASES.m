%% Controlador Preditivo Com Restrições

run Codigos\P1_Modelo_Do_Sistema.m;
close all;

%% Variáveis gerais

A = dados.planta.A;
B = dados.planta.B; 
tau = dados.geral.Ts; 

%pos_limite = dados.geral.guia/2*0.8;
pos_limite = 20/100;
ang_limite = 12*(pi/180);
vel_limite = 45/100;
comando_limite = 12;

ang_inicial = 0*(pi/180);
pos_inicial = 0/100;

pos_spt = 5/100;

tsim=30;

qp_error_count = 0;

%% Variáveis do MPC

[~,nu] = size(B); 

MPC.A = A; 
MPC.B = B;

% Definição das variáveis rastreadas
MPC.Cr = [1 0 0 0; 0 1 0 0];

MPC.Qy = diag([50 10]);
MPC.Qu = 0.001;
MPC.N = 35; 

% Definição das variáveis restringidas
MPC.Cc = [1 0 0 0; 0 1 0 0; 0 0 1 0];

% Definição das restrições
MPC.ycmin = [-pos_limite; -ang_limite; -vel_limite]; MPC.ycmax= [pos_limite; ang_limite; vel_limite]; 
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
yref(1:2:end) = pos_spt;  

lesx(1,:)=x0';
lesy(1,:) = MPC.Cr*lesx(1, :)'; 

x_des = [0 180*pi/180 0 0];

% Opções para o qpOASES e inicialização
options = qpOASES_options('default');
options.enableFarBounds = 0;
options.maxIter = 1000;                
options.terminationTolerance = 1e-3;
options.boundTolerance = 1e-6;
options.enableRegularisation = 1;
QP = [];

sat = @(x, x_max, x_min) min( x_max, max(x_min,x));
%%
for i=1:nt-MPC.N

    if abs(i - (nt-1)/2) < tau/2
        lesx(i,4) = lesx(i,4) - 65*pi/180;
    end

    xplus = RK4_discrete(lesx(i,:),u,tau,dados);
    
    usar_MPC = (abs(xplus(2)-pi) < 8*pi/180) && (qp_error_count == 0) && (abs(xplus(4)) < 90*pi/180);
    if usar_MPC
        yref_pred=yref(i*num_var_reguladas + 1 : (i+MPC.N)*num_var_reguladas);
        err = xplus-x_des;
    
        F=MPC.F1*err'+MPC.F2*yref_pred;
        Bineq=MPC.G1*err'+MPC.G2*MPC.ulast+MPC.G3; 
    
        try
            if i == 1 || isempty(QP)
                [QP, utilde_opt, ~, exitflag, ~, ~] = qpOASES_sequence('i', MPC.H, F, MPC.Aineq, MPC.utildemin, MPC.utildemax, [], Bineq, options);
            else
                [utilde_opt, ~, exitflag, ~, ~] = qpOASES_sequence('h', QP, F, MPC.utildemin, MPC.utildemax, [], Bineq, options);
            end

            u=P_i(1, nu, MPC.N)*utilde_opt;
    
            if exitflag ~= 0 || any(~isfinite(utilde_opt))
                qp_error_count = qp_error_count + 1;
            else
                qp_error_count = 0;
            end
        catch
            qp_error_count = qp_error_count + 1;
        end  
    else
       
       u = EnergySwingUpController(xplus, dados);

       if(abs(xplus(1)) >= 0.18)
           u = -100 * xplus(1);
       else
            qp_error_count = 0;
       end
       u = sat(u,12,-12);   
    end
    
    lesu (i, :)=u'; 
    MPC.ulast=u;
    lesx(i+1, :)=xplus;
    lesy(i+1, :)=MPC.Cr*xplus';
end

Nplot = nt - MPC.N;

posicao = lesx(1:Nplot,1).*100;
angulo = wrapTo360(lesx(1:Nplot,2).*180/pi);
velocidade = lesx(1:Nplot,3).*100;
vel_angular = lesx(1:Nplot,4).*180/pi;
comando = lesu(1:Nplot);
tempo = lest(1:Nplot);

pos_ref = yref(1:2:2*Nplot) * 100;      

%clear A B Bineq err F i lest lesy lesx lesu n nt nu num_var_reguladas Nplot options_qpOASES pos_inicial;
%clear ang_inicial pos_spt QP tau tsim u utilde_opt x0 x_des xplus yref yref_pred sat usar_MPC QP_MAX_FAIL exitflag qp_error_count;

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

plot(tempo,  180+ang_limite*180/pi * ones(size(tempo)),  'r--', 'LineWidth', 2.5)
plot(tempo, 180-ang_limite*180/pi * ones(size(tempo)),  'r--', 'LineWidth', 2.5)

title('Ângulo (°)')
xlabel('Tempo (s)')
ylabel('Ângulo (°)')
legend('Ângulo','Limite Sup','Limite Inf','Location','best')



% ================== VELOCIDADE ==================
figure;
hold on; grid on;

plot(tempo, velocidade, 'LineWidth', 2.5, 'Color', [0 0.45 0.74])

plot(tempo,  vel_limite*100 * ones(size(tempo)),  'r--', 'LineWidth', 2.5)
plot(tempo, -vel_limite*100 * ones(size(tempo)),  'r--', 'LineWidth', 2.5)

title('Velocidade (cm/s)')
xlabel('Tempo (s)')
ylabel('Velocidade (cm/s)')
legend('Velocidade','Limite Sup','Limite Inf','Location','best')



% ================== VELOCIDADE ANGULAR ==================
figure;
hold on; grid on;

plot(tempo, vel_angular, 'LineWidth', 2.5, 'Color', [0 0.45 0.74])

title('Velocidade Angular (°/s)')
xlabel('Tempo (s)')
ylabel('Vel Angular (°/s)')
legend('Vel. Angular','Location','best')

%clear ang_limite ang_ref angulo pos_limite comando comando_limite pos_ref posicao tempo vag_ref vel_ang_limite vel_angular vel_limite vel_ref velocidade;
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