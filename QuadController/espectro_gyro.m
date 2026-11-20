% Espectro de vibracao do giroscopio (etapa 4a/4b do filtro).
%
% No firmware: GYRO_LOG = 1 em QuadController.ino, gravar e ligar o cabo na
% porta USB NATIVA do DUE (a porta de programacao a 57600 nao aguenta 400 Hz).
% Formato enviado a cada tick (mesmo estilo de aquicisao.m), uma linha cada:
%   A, tempo (us), gx, gy, gz brutos (LSB, sem offset), gx, gy, gz filtrados
%
% Como medir: drone PRESO (na mao ou bancada), com helices, throttle perto do
% hover, por ~10 s. Repita com throttle mais baixo e mais alto se puder.
% Mande a figura (ou o .mat salvo) para escolher o filtro e o corte.

clc; clear;

porta   = 'COM17';   % porta USB nativa do DUE (ajuste)
numSec  = 10;        % duracao da coleta
LSB2DPS = 0.0175;    % 500 dps -> 17,5 mdps/LSB

%% Coleta
if exist('serialport', 'file')
    s = serialport(porta, 115200);           % baud ignorado na USB nativa
    configureTerminator(s, 'CR/LF');
    flush(s);
    lerLinha = @() strtrim(readline(s));
else
    s = serial(porta); s.BaudRate = 115200;  %#ok<SERIAL> (MATLAB antigo)
    set(s, 'terminator', 'CR/LF'); fopen(s);
    lerLinha = @() strtrim(fgetl(s));
end

N = ceil(numSec * 400 * 1.1);
t = zeros(N,1); raw = zeros(N,3); filt = zeros(N,3);
i = 0;
disp('Coletando...');
tic;
while toc < numSec && i < N
    if ~strcmp(lerLinha(), 'A'), continue; end   % sincroniza no 'A'
    v = zeros(1,7);
    ok = true;
    for k = 1:7
        x = str2double(lerLinha());
        if isnan(x), ok = false; break; end
        v(k) = x;
    end
    if ~ok, continue; end
    i = i + 1;
    t(i) = v(1) / 1e6;
    raw(i,:) = v(2:4);
    filt(i,:) = v(5:7);
end
if exist('serialport', 'file'), clear s; else, fclose(s); delete(s); end
t = t(1:i) - t(1); raw = raw(1:i,:) * LSB2DPS; filt = filt(1:i,:) * LSB2DPS;

dt = diff(t);
fs = 1 / median(dt);
fprintf('%d amostras, fs = %.1f Hz, dt max = %.2f ms (perdas de amostra se >> 2,5 ms)\n', ...
        i, fs, 1000*max(dt));
save(sprintf('espectro_gyro_%s.mat', datestr(now, 'yyyymmdd_HHMMSS')), 't', 'raw', 'filt', 'fs');

%% Espectro (janela de Hann, sem toolbox)
eixos = {'X (pitch)', 'Y (roll)', 'Z (yaw)'};
n = numel(t);
w = 0.5 - 0.5*cos(2*pi*(0:n-1)'/(n-1));
f = (0:floor(n/2))' * fs / n;
figure('Name', 'Espectro do giroscopio');
for k = 1:3
    for tipo = 1:2
        if tipo == 1, x = raw(:,k); else, x = filt(:,k); end
        X = fft((x - mean(x)) .* w);
        A = 2*abs(X(1:numel(f))) / sum(w);        % amplitude em graus/s
        P(:,tipo) = 20*log10(A + eps);            %#ok<SAGROW>
    end
    subplot(3,1,k);
    plot(f, P(:,1), f, P(:,2)); grid on;
    xlim([0 fs/2]);
    ylabel('dB (graus/s)'); title(eixos{k});
    legend('bruto', 'filtrado');

    % 3 maiores picos acima de 20 Hz no bruto (candidatos a notch)
    idx = find(f > 20);
    [~, ord] = sort(P(idx,1), 'descend');
    picos = [];
    for j = ord'
        fj = f(idx(j));
        if all(abs(picos - fj) > 5), picos(end+1) = fj; end %#ok<AGROW>
        if numel(picos) == 3, break; end
    end
    fprintf('%s: picos em %s Hz\n', eixos{k}, mat2str(round(picos)));
end
xlabel('Frequencia (Hz)');
