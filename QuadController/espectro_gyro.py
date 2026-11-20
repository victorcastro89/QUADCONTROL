#!/usr/bin/env python3
"""
Espectro de vibracao do giroscopio (etapa 4b do filtro).

No firmware: GYRO_LOG = 1 em QuadController.ino, gravar e ligar o cabo na porta
USB NATIVA do DUE (a porta de programacao a 57600 nao aguenta 400 Hz).
Formato enviado a cada tick, uma linha cada:
    A / tempo (us) / gx, gy, gz brutos (LSB, sem offset) / gx, gy, gz filtrados

Como medir: drone PRESO (na mao ou bancada), com helices, throttle perto do
hover, por ~10 s. Se puder, repita com throttle mais baixo e mais alto.

Instalar:  pip install pyserial numpy matplotlib
Coletar:   python espectro_gyro.py --porta COM17          (Windows)
           python espectro_gyro.py --porta /dev/ttyACM0   (Linux/macOS)
Reanalisar um arquivo ja salvo:
           python espectro_gyro.py --arquivo espectro_gyro_20261120_153000.csv

Gera um CSV com os dados e um PNG com o grafico. Mande os dois para escolher o
filtro (GYRO_FILTER_TYPE / GYRO_LPF_HZ / GYRO_NOTCH_HZ em GyroFilter.h).
"""
import argparse
import csv
import datetime
import sys
import time

import numpy as np

LSB2DPS = 0.0175          # 500 dps -> 17,5 mdps/LSB
EIXOS = ["X (pitch)", "Y (roll)", "Z (yaw)"]
CAMPOS = ["t_us", "raw_x", "raw_y", "raw_z", "filt_x", "filt_y", "filt_z"]


def ler_amostras(linhas):
    """Converte as linhas do firmware em amostras (7 numeros cada).

    Sincroniza na linha 'A'; se uma amostra vier truncada ou com lixo, ela e
    descartada e a leitura volta a procurar o proximo 'A'.
    """
    linhas = iter(linhas)
    for linha in linhas:
        if linha.strip() != "A":
            continue
        valores = []
        for linha in linhas:
            linha = linha.strip()
            if linha == "A":            # amostra truncada: recomeca daqui
                valores = None
                break
            try:
                valores.append(float(linha))
            except ValueError:
                valores = None
                break
            if len(valores) == 7:
                break
        if valores and len(valores) == 7:
            yield valores


def linhas_da_serial(porta, segundos):
    import serial  # pyserial
    with serial.Serial(porta, 115200, timeout=1) as s:   # baud ignorado na USB nativa
        s.reset_input_buffer()
        fim = time.time() + segundos
        while time.time() < fim:
            yield s.readline().decode("ascii", errors="ignore")


def coletar(porta, segundos):
    print(f"Coletando {segundos} s de {porta}...")
    dados = np.array(list(ler_amostras(linhas_da_serial(porta, segundos))))
    if len(dados) < 100:
        sys.exit("Poucas amostras recebidas. Confira GYRO_LOG = 1, a porta USB nativa e o nome da porta.")
    return dados


def salvar_csv(dados, nome):
    with open(nome, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(CAMPOS)
        w.writerows(dados.tolist())
    print(f"Dados salvos em {nome}")


def carregar_csv(nome):
    return np.loadtxt(nome, delimiter=",", skiprows=1)


def espectro(x, fs):
    """Amplitude (graus/s) e frequencias, com janela de Hann."""
    n = len(x)
    w = np.hanning(n)
    X = np.fft.rfft((x - x.mean()) * w)
    amp = 2 * np.abs(X) / w.sum()
    return np.fft.rfftfreq(n, 1 / fs), amp


def picos(f, amp, n=3, fmin=20.0, separacao=10.0, faixa_db=40.0):
    """Os n maiores maximos locais acima de fmin, separados por 'separacao' Hz.

    Ignora picos mais de 'faixa_db' abaixo do maior (vazamento espectral/ruido).
    """
    cand = [i for i in range(1, len(amp) - 1)
            if f[i] > fmin and amp[i] >= amp[i - 1] and amp[i] >= amp[i + 1]]
    cand.sort(key=lambda i: amp[i], reverse=True)
    if cand:
        limite = amp[cand[0]] / 10 ** (faixa_db / 20)
        cand = [i for i in cand if amp[i] >= limite]
    escolhidos = []
    for i in cand:
        if all(abs(f[i] - f[j]) > separacao for j in escolhidos):
            escolhidos.append(i)
        if len(escolhidos) == n:
            break
    return [(f[i], amp[i]) for i in escolhidos]


def analisar(dados):
    t = (dados[:, 0] - dados[0, 0]) / 1e6
    raw = dados[:, 1:4] * LSB2DPS
    filt = dados[:, 4:7] * LSB2DPS
    dt = np.diff(t)
    fs = 1 / np.median(dt)
    print(f"{len(t)} amostras, fs = {fs:.1f} Hz, dt max = {1000 * dt.max():.2f} ms "
          f"(se for >> 2,5 ms, houve perda de amostras)")
    resultado = []
    for k in range(3):
        f, a_raw = espectro(raw[:, k], fs)
        _, a_filt = espectro(filt[:, k], fs)
        pk = picos(f, a_raw)
        txt = ", ".join(f"{fp:.0f} Hz ({20 * np.log10(ap + 1e-12):.0f} dB)" for fp, ap in pk)
        print(f"{EIXOS[k]}: picos em {txt}")
        resultado.append((f, a_raw, a_filt, pk))
    return fs, resultado


def plotar(fs, resultado, nome_png, mostrar):
    import matplotlib
    if not mostrar:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axs = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
    for k, (f, a_raw, a_filt, pk) in enumerate(resultado):
        ax = axs[k]
        ax.plot(f, 20 * np.log10(a_raw + 1e-12), label="bruto")
        ax.plot(f, 20 * np.log10(a_filt + 1e-12), label="filtrado")
        for fp, ap in pk:
            ax.annotate(f"{fp:.0f} Hz", (fp, 20 * np.log10(ap + 1e-12)),
                        textcoords="offset points", xytext=(0, 5), ha="center", fontsize=8)
        ax.set_title(EIXOS[k])
        ax.set_ylabel("dB (graus/s)")
        ax.grid(True)
        ax.legend(loc="upper right")
    axs[-1].set_xlabel("Frequencia (Hz)")
    axs[-1].set_xlim(0, fs / 2)
    fig.suptitle("Espectro do giroscopio")
    fig.tight_layout()
    fig.savefig(nome_png, dpi=120)
    print(f"Grafico salvo em {nome_png}")
    if mostrar:
        plt.show()


def main():
    p = argparse.ArgumentParser(description="Espectro de vibracao do giroscopio do QuadController.")
    origem = p.add_mutually_exclusive_group(required=True)
    origem.add_argument("--porta", help="porta USB nativa do DUE (ex.: COM17, /dev/ttyACM0)")
    origem.add_argument("--arquivo", help="CSV salvo antes, para reanalisar sem o drone")
    p.add_argument("--segundos", type=float, default=10, help="duracao da coleta (padrao 10)")
    p.add_argument("--sem-janela", action="store_true", help="so salva o PNG, sem abrir a janela")
    args = p.parse_args()

    if args.porta:
        dados = coletar(args.porta, args.segundos)
        base = "espectro_gyro_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        salvar_csv(dados, base + ".csv")
    else:
        dados = carregar_csv(args.arquivo)
        base = args.arquivo.rsplit(".", 1)[0]

    fs, resultado = analisar(dados)
    plotar(fs, resultado, base + ".png", mostrar=not args.sem_janela)


if __name__ == "__main__":
    main()
