# =============================================================================
# valuta.py — valuta una policy SAC gia' addestrata, senza rifare il training
# -----------------------------------------------------------------------------
# Uso:
#   python valuta.py                                  (usa ./best_sac_fan/best_model.zip)
#   python valuta.py percorso\del\modello.zip         (qualsiasi altro modello)
#
# Stampa le metriche su episodi nominali e randomizzati e salva due grafici:
#   - episodio_esempio.png  : forza, comando e inclinazione in un episodio
#   - learning_curve.png    : curva di apprendimento da ./logs/evaluations.npz
# =============================================================================
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from stable_baselines3 import SAC

from env_fan import FanEnv

percorso = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("best_sac_fan/best_model.zip")
# NB: indicare SEMPRE l'estensione .zip. Senza, SB3 prova ad aprire il percorso cosi' com'e':
# se esiste una CARTELLA con lo stesso nome (es. uno zip estratto), Windows da' PermissionError.
model = SAC.load(str(percorso), device="cpu")
print(f"Modello: {percorso}")


def evaluate(dr, n_ep=20):
    """Esegue n_ep episodi deterministici e restituisce le metriche medie."""
    rms, du, spegn, err_medio = [], [], [], []
    for ep in range(n_ep):
        env = FanEnv(domain_randomization=dr)
        obs, _ = env.reset(seed=100 + ep)              # stessi seed per ogni modello -> confronto equo
        E, C, M = [], [], []
        for _ in range(env.max_episode_steps):
            a, _ = model.predict(obs, deterministic=True)
            obs, r, term, trunc, info = env.step(a)
            E.append(info["force_ref"] - info["force"])
            C.append(info["ESC"])
            M.append(info["in_moto"])
            if term or trunc:
                break
        E, C, M = np.array(E), np.array(C), np.array(M)
        rms.append(np.sqrt(np.mean(E[100:] ** 2)))      # escludo i primi 2 s (avvio da fermo)
        err_medio.append(np.mean(E[100:]))              # errore medio: misura l'offset che resta
        du.append(np.mean(np.abs(np.diff(C))))
        spegn.append(int(np.sum(M[:-1] & ~M[1:])))
    return np.mean(rms), np.std(rms), np.mean(err_medio), np.mean(du), np.mean(spegn)


for nome, dr in [("nominale", False), ("randomizzato", True)]:
    m, s, em, d, k = evaluate(dr)
    print(f"[{nome:12s}] RMS errore: {m:.3f} ± {s:.3f} N | errore medio: {em:+.3f} N | "
          f"mean|Δc|: {d:.4f} | spegnimenti/episodio: {k:.1f}")

# ---------------- Episodio di esempio (stesso seed usato nei confronti precedenti) ----------------
env = FanEnv(domain_randomization=True)
obs, _ = env.reset(seed=105)
L = []
for k in range(env.max_episode_steps):
    a, _ = model.predict(obs, deterministic=True)
    obs, r, term, trunc, info = env.step(a)
    L.append((k * env.dt, info["force"], info["ESC"], info["angle"]))
L = np.array(L)
fig, ax = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
ax[0].plot(L[:, 0], L[:, 1], lw=0.8); ax[0].axhline(env.force_ref, color="k", ls="--")
ax[0].set_ylabel("F load cell [N]"); ax[0].set_title(f"Episodio di esempio — {percorso.name}")
ax[1].plot(L[:, 0], L[:, 2], lw=0.8); ax[1].set_ylabel("comando c (0..1)")
ax[2].plot(L[:, 0], L[:, 3]); ax[2].set_ylabel("inclinazione [deg]"); ax[2].set_xlabel("t [s]")
for a_ in ax: a_.grid(True)
plt.tight_layout(); plt.savefig("episodio_esempio.png", dpi=110)
print("Salvato episodio_esempio.png")

# ---------------- Curva di apprendimento ----------------
log = Path("logs/evaluations.npz")
if log.exists():
    data = np.load(log)
    t, res = data["timesteps"], data["results"]
    plt.figure(figsize=(8, 5))
    plt.plot(t, res.mean(axis=1), marker="o", label="Reward medio (5 ep)")
    plt.fill_between(t, res.mean(axis=1) - res.std(axis=1), res.mean(axis=1) + res.std(axis=1),
                     alpha=0.25, label="±1σ")
    plt.title("Curva di apprendimento – EvalCallback")
    plt.xlabel("Timesteps"); plt.ylabel("Reward episodio"); plt.grid(True); plt.legend()
    plt.tight_layout(); plt.savefig("learning_curve.png", dpi=150)
    print("Salvato learning_curve.png")

plt.show()
