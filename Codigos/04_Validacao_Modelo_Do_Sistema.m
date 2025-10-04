clear; clc; close all;
% Limpa todas as variáveis da memória, limpa o console e fecha figuras abertas

porta_serial = 'COM14'; % Define a porta serial (verifique se está correta para o seu sistema)
baud_rate = 115200;     % Define a taxa de comunicação serial
s = serialport(porta_serial, baud_rate); % Cria o objeto de comunicação serial
configureTerminator(s, "LF");           % Define o terminador de linha como 'line feed' (\n)
flush(s);                               % Limpa o buffer da serial (remove dados pendentes)

% Inicialização dos vetores que irão armazenar os dados vindos do Arduino
tempo = [];
setpoint = [];
rpm = [];

tempo_total = 5; % Tempo total de aquisição em segundos

t_inicio = tic; % Marca o tempo inicial com alta precisão (cronômetro interno)

% Loop principal: executa enquanto o tempo decorrido for menor que o tempo total
while toc(t_inicio) < tempo_total
    if s.NumBytesAvailable > 0 % Verifica se há dados disponíveis na porta serial
        raw_data = readline(s); % Lê uma linha de dados da serial

        dados = strsplit(strtrim(raw_data), ','); % Divide a string usando vírgula como separador

        if length(dados) == 3 % Verifica se a linha possui exatamente 3 elementos
            t = str2double(dados{1}); % Converte o tempo (1º valor) para número
            sp = str2double(dados{2}); % Converte o setpoint (2º valor)
            v = str2double(dados{3});  % Converte o RPM (3º valor)

            % Verifica se nenhum dos valores é NaN (erro de conversão)
            if ~any(isnan([t, sp, v]))
                tempo(end+1) = t;     % Armazena o tempo no vetor
                setpoint(end+1) = sp; % Armazena o setpoint
                rpm(end+1) = v;       % Armazena a velocidade (RPM)
            end
        else
        end
    end
end

clear s; % Encerra a comunicação com a porta serial

% Plotagem dos resultados
figure; % Cria uma nova figura
plot(tempo, rpm, 'b', 'LineWidth', 1.5); % Plota o RPM em azul
hold on; % Mantém o gráfico para adicionar o setpoint
plot(tempo, setpoint, 'r--', 'LineWidth', 1.5); % Plota o setpoint em vermelho tracejado
xlabel('Tempo (s)'); % Rótulo do eixo X
ylabel('Velocidade (RPM)'); % Rótulo do eixo Y
legend('RPM', 'Setpoint'); % Legenda dos dados
title('Resposta do sistema com controlador - Arduino'); % Título do gráfico
grid on; % Adiciona grade ao gráfico