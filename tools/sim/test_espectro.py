"""Teste do espectro_gyro.py sem hardware: um 'DUE falso' numa porta serial
virtual (pty) envia o formato do firmware com vibracoes conhecidas.
Uso (Linux/macOS): python tools/sim/test_espectro.py
"""
import os, sys, threading, time, tempfile
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "QuadController"))
import espectro_gyro as eg

FS = 400.0
TONS = {0: [(85, 300), (170, 120)], 1: [(120, 200)], 2: [(60, 80)]}   # eixo: [(Hz, LSB)]

def amostra(i):
    t = i / FS
    raw = []
    for k in range(3):
        v = 50 * np.sin(2 * np.pi * 2 * t)  # movimento lento de 2 Hz
        v += sum(a * np.sin(2 * np.pi * f * t) for f, a in TONS[k])
        raw.append(int(round(v)))
    return int(t * 1e6), raw

def fake_due(fd, n):
    for i in range(n):
        t_us, raw = amostra(i)
        linhas = ["A", str(t_us)] + [str(v) for v in raw] + [f"{v:.2f}" for v in raw]
        if i % 500 == 250:
            linhas = linhas[:4]          # amostra truncada (ruido na USB)
        if i % 700 == 350:
            linhas[3] = "x#!"            # lixo
        os.write(fd, ("\r\n".join(linhas) + "\r\n").encode())
        if i % 40 == 0:
            time.sleep(0.001)

def main():
    falhas = 0
    def check(ok, msg):
        nonlocal falhas
        print(("OK   " if ok else "FALHA ") + msg)
        falhas += 0 if ok else 1

    mestre, escravo = os.openpty()
    n = int(5 * FS)
    th = threading.Thread(target=fake_due, args=(mestre, n), daemon=True)
    threading.Timer(0.3, th.start).start()           # comeca depois de abrir a porta
    dados = eg.coletar(os.ttyname(escravo), 3.0)
    check(len(dados) > 0.95 * n and len(dados) < n, f"recebeu {len(dados)} de {n} amostras (corrompidas descartadas)")
    check(np.all(np.diff(dados[:, 0]) > 0), "tempos crescentes, sem amostra misturada")

    with tempfile.TemporaryDirectory() as d:
        nome = os.path.join(d, "teste.csv")
        eg.salvar_csv(dados, nome)
        dados2 = eg.carregar_csv(nome)
        check(np.allclose(dados, dados2), "CSV salvo e recarregado igual")
        fs, res = eg.analisar(dados2)
        check(abs(fs - FS) < 1, f"fs estimada = {fs:.1f} Hz")
        for k in range(3):
            achados = sorted(round(fp) for fp, _ in res[k][3][:len(TONS[k])])
            esperados = sorted(f for f, _ in TONS[k])
            check(all(abs(a - e) <= 1 for a, e in zip(achados, esperados)),
                  f"{eg.EIXOS[k]}: picos {achados} Hz (esperado {esperados})")
        png = os.path.join(d, "teste.png")
        eg.plotar(fs, res, png, mostrar=False)
        check(os.path.getsize(png) > 10000, "PNG gerado")
    sys.exit(falhas)

if __name__ == "__main__":
    main()
