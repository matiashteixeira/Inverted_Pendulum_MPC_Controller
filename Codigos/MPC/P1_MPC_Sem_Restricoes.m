%% Controlador Preditivo Sem Restrições

A = planta.A;
B = planta.B;
tau = dados.geral.Ts;

%A=[0 1 0; 0 0 1; 0 0 0]; B=[0;0;1]; tau=0.1;

[n,nu]=size(B);
MPC.A = A; MPC.B = B;
MPC.Cr=[1 0 0 0];
MPC.Qy=100;
MPC.Qu=1;
MPC.N=20;

%--------------------------------------------
[H,F1,F2,F3]=compute_cost_matrices(MPC);
KN=-P_i(1,nu,MPC.N)*(H\F1);
GN=-P_i(1,nu,MPC.N)*(H\F2);
%--------------------------------------------
x0=[dados.geral.inicial.x0;dados.geral.inicial.theta0*pi/180;dados.geral.inicial.x_dot0;dados.geral.inicial.theta_dot0];
tsim=20;
lest=(0:tau:tsim)'; nt= size(lest,1);
lesx=zeros(nt,length(B)); lesy=zeros(nt,1); lesu=zeros(nt,1);
yref=zeros(1,length(lest))';
%%
lesx(1,:) = x0';
lesy(1)=MPC.Cr*lesx(1,:)';

for i=1:nt-1
    yref_pred=yref(i+1:i+MPC.N);
    u=KN*lesx(i,:)'+GN*yref_pred;
    lesu(i,:)=u';
    xplus=MPC.A*lesx(i,:)'+MPC.B*u;
    lesx(i+1,:)=xplus';
    lesy(i+1)=MPC.Cr*xplus;
end

controlador.MPC = MPC;

%% Plot dos resultados

subplot(2,1,1);
stairs(lest,lesu); grid on;
title('Sinal de Comando');

subplot(2,1,2);
plot(lest,lesx(:,1),lest,yref(1:(length(lest)))); grid on;
title('Trajetory Tracking');