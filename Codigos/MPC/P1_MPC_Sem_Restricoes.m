%% Controlador Preditivo Sem Restrições

A = dados.planta.A;
B = dados.planta.B;
tau = dados.geral.Ts;

[n,nu]=size(B);
MPC.A = A; MPC.B = B;
MPC.Cr=[1 0 0 0
        0 1 0 0];
MPC.Qy=[1 0; 0 1];
MPC.Qu=100;
MPC.N=20;

%--------------------------------------------
[H,F1,F2,~]=compute_cost_matrices(MPC);
%KN=-P_i(1,nu,MPC.N)*(H\F1);
KN=[7.3539 -113.2320   59.0265  -17.2086];
GN=-P_i(1,nu,MPC.N)*(H\F2);
%--------------------------------------------
x0=[0;dados.geral.inicial.theta0*pi/180;dados.geral.inicial.x_dot0;dados.geral.inicial.theta_dot0];
tsim=30;
lest=(0:tau:tsim)'; nt= size(lest,1);
lesx=zeros(nt,length(B)); lesy=zeros(nt,size(MPC.Cr,1)); lesu=zeros(nt,1);

    yref = zeros(length(lest)*2,1);             % inicializa vetor coluna
    yref(1:2:end) = dados.geral.spt/100;  % índices ímpares
    yref(2:2:end) = pi;                   % índices pares

lesx(1,:) = x0';
lesy(1,:)=(MPC.Cr*lesx(1,:)')';


for i=1:nt-MPC.N
    yref_pred=yref(i+1:i+MPC.N*size(MPC.Cr,1));
    u=KN*lesx(i,:)'+GN*yref_pred;
    lesu(i,:)=u';
    xplus=MPC.A*lesx(i,:)'+MPC.B*u;
    lesx(i+1,:)=xplus';
    lesy(i+1,:)=MPC.Cr*xplus;
end

controlador.MPC = MPC;

posicao = lesx(:,1).*100;
angulo = lesx(:,2).*180/pi;
velocidade = lesx(:,3).*100;
vel_angular = lesx(:,4).*180/pi;

%% Plot dos resultados

figure;
stairs(lest,lesu); grid on;
title('Sinal de Comando');

figure;
% 1. Posição do Carrinho
subplot(2, 2, 1);
stairs(lest, posicao, 'k-');
title('Posição do Carrinho');
xlabel('Tempo (s)');
ylabel('Posição (cm)');
grid on;

% 2. Ângulo do Pêndulo
subplot(2, 2, 2);
stairs(lest, angulo, 'k-');
title('Ângulo do Pêndulo');
xlabel('Tempo (s)');
ylabel('Ângulo (°)');
grid on;

% 3. Velocidade do Carrinho
subplot(2, 2, 3);
stairs(lest, velocidade, 'k-');
title('Velocidade do Carrinho');
xlabel('Tempo (s)');
ylabel('Velocidade (cm/s)');
grid on;

% 4. Velocidade Angular do Pêndulo
subplot(2, 2, 4);
stairs(lest, vel_angular, 'k-');
title('Velocidade Angular');
xlabel('Tempo (s)');
ylabel('Velocidade Ang. (°/s)');
grid on;

%clear A B F1 F2 F3 GN H i KN lest lesu lesx lesy MPC n nt nu tau tsim var_rastreadas x0 yref yref_pred;
