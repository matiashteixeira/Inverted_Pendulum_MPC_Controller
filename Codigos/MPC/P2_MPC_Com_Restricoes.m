%% Controlador Preditivo Com Restrições

%clear; clc; close all;

%% Declaração de variáveis

A = planta.A;
B = planta.B; 
limite = dados.geral.guia/2;

tau = dados.geral.Ts; 
[n,nu] = size(B); 

MPC.A = A; 
MPC.B = B;
MPC.Cr = [1 0 0 0]; 

MPC.Qy = 1e9; MPC.Qu=1; 
MPC.N = 150; 

MPC.Cc = [1 0 0 0];
MPC.ycmin = -35; MPC.ycmax=35; 
MPC.umin = -100; MPC.umax = 100; 
MPC.ulast  =0; MPC.deltamin=-100; MPC.deltamax=100;

%-------------------------
[MPC]=compute_MPC_Matrices(MPC);
MPC.H = (MPC.H + MPC.H') / 2;
%-------------------------

x0=[0;0;0;0];
tsim=20;
lest=(0:tau:tsim)';
nt=size(lest, 1);
lesx=zeros(nt,length(B)); lesy=zeros(nt, 1); lesu=zeros(nt, 1);
yref = ones(nt,1);
%----
lesx(1,:)=x0';
lesy(1) = MPC.Cr*lesx(1, :)'; 
for i=1:nt-1
    yref_pred=yref(i+1:i+MPC.N);

    F=MPC.F1*lesx(i,:)'+MPC.F2*yref_pred;
    Bineq=MPC.G1*lesx(i, :)'+MPC.G2*MPC.ulast+MPC.G3; 

    options=optimset('maxIter', 100000); 
    utilde_opt=quadprog(MPC.H, F, MPC.Aineq, Bineq,... 
        [], [], MPC.utildemin, MPC.utildemax);

    u=P_i(1, nu, MPC.N)*utilde_opt;
    lesu (i, :)=u'; 
    MPC.ulast=u;
    xplus=MPC.A*lesx(i, :)'+MPC.B*u;
    lesx(i+1, :)=xplus';
    lesy(i+1)=MPC.Cr*xplus;

end

%% Plot dos resultados

subplot(1,2,1);
stairs(lest,lesu); grid on;
title('Sinal de Comando');

subplot(1,2,2);
plot(lest,lesx(:,1),lest,yref(1:(length(lest)))); grid on;
title('Trajetory Tracking');

