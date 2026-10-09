# analizza_prova.py — metriche e grafico di una o piu' prove salvate da monitor_stm32.py
# Uso: python analizza_prova.py prova1.csv [prova2.csv ...] [-o grafico.png]
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

def carica(p):
    d = np.genfromtxt(p, delimiter=",", names=True)
    return d["t_ms"] / 1000, d["F_mN"] / 1000, d["c_x1000"] / 1000, d["u_us"], d["inferenza_us"], d["eta_ms"]

argv = sys.argv[1:]
uscita = "analisi_prove_stm32.png"
if "-o" in argv:
    k = argv.index("-o"); uscita = argv[k + 1]; del argv[k:k + 2]

fig, ax = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
for p in argv:
    t, F, c, us, inf, eta = carica(p)
    dt = np.diff(t)
    # riferimento: stimato dalla mediana della forza dopo 10 s (non lo so a priori: dipende dal firmware caricato)
    ref_stim = np.median(F[t > 10])
    ref = 1.15 if abs(ref_stim - 1.15) < abs(ref_stim - 1.0) else 1.0
    e = ref - F
    m = t > 2
    print(f"\n=== {p}")
    print(f"durata {t[-1]:.1f} s | righe {len(t)} | passo: mediana {np.median(dt)*1000:.0f} ms, max {dt.max()*1000:.0f} ms "
          f"| righe perse (salti > 30 ms): {int(np.sum(dt > 0.03))}")
    print(f"forza mediana dopo 10 s: {ref_stim:.3f} N  -> firmware con riferimento {ref} N")
    print(f"dopo 2 s : RMS errore {np.sqrt(np.mean(e[m]**2)):.3f} N | errore medio {np.mean(e[m]):+.3f} N | "
          f"c medio {np.mean(c[m]):.3f} | impulso medio {np.mean(us[m]):.0f} us")
    for a, b in [(0, 2), (2, 5), (5, 10), (10, 30), (30, 60), (60, t[-1] + 1)]:
        w = (t >= a) & (t < b)
        if w.sum() > 5:
            print(f"  {a:4.0f}-{min(b, t[-1]):5.1f} s: RMS errore {np.sqrt(np.mean(e[w]**2)):.3f} N | "
                  f"F min/max {F[w].min():.2f}/{F[w].max():.2f} N | c min/max {c[w].min():.2f}/{c[w].max():.2f}")
    # frequenza dominante dell'oscillazione nei primi 10 s (FFT della forza senza media)
    w = t < 10
    x = F[w] - F[w].mean()
    f = np.fft.rfftfreq(len(x), d=np.median(dt)); A = np.abs(np.fft.rfft(x))
    k = np.argmax(A[1:]) + 1
    print(f"  oscillazione dominante 0-10 s: {f[k]:.2f} Hz")
    print(f"inferenza: {inf.min():.0f}-{inf.max():.0f} us | eta' campione: mediana {np.median(eta):.0f} ms, max {eta.max():.0f} ms")
    ax[0].plot(t, F, lw=0.7, label=Path(p).stem)
    ax[1].plot(t, c, lw=0.7, label=Path(p).stem)
ax[0].axhline(1.15, color="k", ls="--", lw=0.8); ax[0].set_ylabel("F cella [N]"); ax[0].grid(); ax[0].legend()
ax[1].set_ylabel("comando c"); ax[1].set_xlabel("t [s]"); ax[1].grid()
plt.tight_layout(); plt.savefig(uscita, dpi=110)
print(f"\nGrafico: {uscita}")
