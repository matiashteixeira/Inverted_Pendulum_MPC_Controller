%% Controlador LQR

% Definição das matrizes de ponderamento do LQR
%controlador.lqr.Q = diag([50 200 10 20]);controlador.lqr.R = 0.005;
controlador.lqr.Q = diag([200 1000 0 0]);controlador.lqr.R = 0.035;
controlador.lqr.K = lqr(planta.A,planta.B,controlador.lqr.Q,controlador.lqr.R);

%% Controlador baseado na lei de Energia

controlador.energy.k = 1.2;
controlador.energy.n = 3;

%% Simulação em malha fechada

m = dados.pendulo.m;
M = dados.carro.m;
g = dados.geral.g;
I = dados.pendulo.I;
l = dados.pendulo.l;
b = dados.pendulo.b;
c = dados.carro.c;
E_des = dados.pendulo.E_des;
Ts = dados.geral.Ts; % Período de Amostragem
Tf = dados.geral.Tf; % Tempo final da simulação

x(1,:) = [dados.geral.inicial.x0 dados.geral.inicial.theta0*pi/180 dados.geral.inicial.x_dot0 dados.geral.inicial.theta_dot0]; % Estado Inicial
estado_novo = x(1,:)';
u_volt(1) = 0;
u_force(1) = 0;
t(1) = 0;

x_des = [dados.geral.spt/100; pi; 0; 0];            % Estado desejado

i = 1;
sat = @(x, x_max, x_min) min( x_max, max(x_min,x));

for k = 0:Ts:Tf
    
    estado_novo = labInt.RK4_discrete(estado_novo',u_force(i),Ts,dados)';
    theta = estado_novo(2);
    x_dot = estado_novo(3);
    theta_dot = estado_novo(4);
    
    % O controlador por energia precisa utilizar apenas ângulos positivos,
    % por isso deve-se o correspondente positivo do ângulo (360-theta)
    if theta < 0
       th = 2*pi-abs(theta);
       estado_atualizado = [estado_novo(1); th; x_dot; theta_dot];
    else
       estado_atualizado = estado_novo;
    end
    
    x(i+1,:) = estado_atualizado'; % Dados para plot

    t(i+1) = k;

    % Escolha do controlador adequado
    if (abs(x_des(2)-estado_atualizado(2)))*(180/pi) <= dados.geral.angulo_troca
        % Controlador LQR
        disp(k);
        u_volt(i+1) = -controlador.lqr.K * (estado_atualizado - x_des);
        u_volt(i+1) = sat(u_volt(i+1),12,-12);
        u_force(i+1) = labInt.Volt2Force(u_volt(i+1),x_dot,dados.motor);
        
    else     
        % Controlador por Energia
        E(i) = m*g*l*(1-cos(theta)) + (1/2)*(I + m*l^2)*(theta_dot^2); % Energia do Pêndulo
        x_2dot = (E(i)-E_des)*sign(theta_dot*cos(theta));
        x_2dot = controlador.energy.k*g*(sat(x_2dot, controlador.energy.n*g, -controlador.energy.n*g));
        
        u_force(i+1) = 0*x_dot + (M+m)*(x_2dot) - m*l*(cos(theta))*(b*theta_dot + m*l*x_2dot*cos(theta) + m*g*l*sin(theta))/(I+m*l^2) - m*l*((theta_dot)^2)*sin(theta);
        u_volt(i+1) = labInt.Force2Volt(u_force(i+1), x_dot, dados.motor);
        u_volt(i+1) = sat(u_volt(i+1),12,-12);
        u_force(i+1) = labInt.Volt2Force(u_volt(i+1),x_dot,dados.motor);
    end

    i = i+1;
end

simulacao.tempo = t;
simulacao.angulo = (180/pi).* x(:,2);
simulacao.velocidade_pendulo = (180/pi).*x(:,4);
simulacao.posicao = x(:,1)*100;
simulacao.velocidade_carro = x(:,3);
simulacao.u_force = u_force;
simulacao.u_volt = u_volt;

clear b c g E E_des I l m M theta theta_dot x_2dot x_dot k th u_force u_volt i estado_novo estado_atualizado t Tf Ts x_des sat;

%% Plot dos dados

subplot(2,2,1);
stairs(simulacao.tempo, simulacao.angulo,'LineWidth',2); grid on;
title('Posição Angular do Pêndulo [°]');

subplot(2,2,2);
stairs(simulacao.tempo, simulacao.velocidade_pendulo,'LineWidth',2); grid on;
title('Velocidade Angular do Pêndulo [rad/s]');

subplot(2,2,3);
stairs(simulacao.tempo, simulacao.posicao,'LineWidth',2); grid on;
title('Posição Linear do Carro [cm]');

subplot(2,2,4);
stairs(simulacao.tempo, simulacao.velocidade_carro,'LineWidth',2); grid on;
title('Velocidade Linear do Carro [m/s]');

figure;

subplot(1,2,1);
stairs(simulacao.tempo,simulacao.u_force,'LineWidth',2); grid on;
title('Comando - Força [F]');

subplot(1,2,2);
stairs(simulacao.tempo,simulacao.u_volt,'LineWidth',2); grid on;
title('Comando - Tensão [V]');

%% Animação do Pêndulo
% hf = figure();
% for i = 1:8:length(x)
%    labInt.IP_Animation(simulacao.posicao(i),simulacao.angulo(i), dados);
%    pause(0.01);
%   % movieVector(i) =  getframe(hf);
%    hold off
% end

% %% Save the movie
% myWriter = VideoWriter('IP_SwingUp', 'Motion JPEG AVI');
% %myWriter = VideoWriter('IP_animation1', 'MPEG-4');
% myWriter.Quality    = 100;
% myWritter.FrameRate = 180;
% 
% % Open the VideoWriter object, write the movie, and class the file
% open(myWriter);
% writeVideo(myWriter, movieVector);
% close(myWriter);
