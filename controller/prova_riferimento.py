# =============================================================================
# prova_riferimento.py — la policy addestrata a 1.15 N funziona con un altro riferimento?
# Stesse metriche e stessi seed di valuta.py; cambia SOLO force_ref dell'ambiente.
# Uso: python prova_riferimento.py ..\models\sac_F.zip 1.15 0.8
# =============================================================================
import sys
import numpy as np
from stable_baselines3 import SAC
from env_fan import FanEnv

model = SAC.load(sys.argv[1], device="cpu")
riferimenti = [float(x) for x in sys.argv[2:]] or [1.15, 0.8]

def valuta(ref, dr, n_ep=20):
    rms, em, spegn, sat_bassa = [], [], [], []
    for ep in range(n_ep):
        env = FanEnv(domain_randomization=dr, force_ref=ref)
        obs, _ = env.reset(seed=100 + ep)
        E, M, C = [], [], []
        for _ in range(env.max_episode_steps):
            a, _ = model.predict(obs, deterministic=True)
            obs, r, term, trunc, info = env.step(a)
            E.append(info["force_ref"] - info["force"]); M.append(info["in_moto"]); C.append(info["ESC"])
            if term or trunc:
                break
        E, M, C = np.array(E), np.array(M), np.array(C)
        rms.append(np.sqrt(np.mean(E[100:] ** 2)))
        em.append(np.mean(E[100:]))
        spegn.append(int(np.sum(M[:-1] & ~M[1:])))
        sat_bassa.append(np.mean(C[100:] <= env.c_min + 1e-3))   # quota di tempo al comando minimo
    return np.mean(rms), np.std(rms), np.mean(em), np.mean(spegn), 100 * np.mean(sat_bassa)

for ref in riferimenti:
    for nome, dr in [("nominale", False), ("randomizzato", True)]:
        m, s, e, k, sb = valuta(ref, dr)
        print(f"ref {ref:.2f} N [{nome:12s}] RMS {m:.3f} ± {s:.3f} N | errore medio {e:+.3f} N | "
              f"spegnimenti/ep {k:.1f} | tempo al comando minimo {sb:.1f} %")
