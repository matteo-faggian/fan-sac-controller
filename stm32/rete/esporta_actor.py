# =============================================================================
# esporta_actor.py — trasforma l'actor SAC (Stable-Baselines3) in C per lo STM32
# -----------------------------------------------------------------------------
# Produce:
#   actor_weights.h : tutti i pesi e bias come array C costanti (finiscono in FLASH)
#   test_vectors.h  : 100 osservazioni di test + l'azione calcolata da PyTorch,
#                     per verificare che il C dia lo stesso risultato
# Uso: python esporta_actor.py percorso\modello.zip
#      es. python esporta_actor.py ..\..\models\sac_F.zip
# =============================================================================
import sys
import numpy as np
import torch
from stable_baselines3 import SAC

model = SAC.load(sys.argv[1] if len(sys.argv) > 1 else "best_model.zip", device="cpu")
actor = model.actor

# --- 1. Estraggo i pesi. In inferenza deterministica servono solo latent_pi e mu: ---
#     azione = tanh( mu( relu(W2 relu(W1 x + b1) + b2) ) )
#     log_std serve solo per campionare in training: qui non lo esporto.
L1 = actor.latent_pi[0]   # Linear 7 -> 256
L2 = actor.latent_pi[2]   # Linear 256 -> 256
MU = actor.mu             # Linear 256 -> 1
strati = [("W1", L1.weight), ("B1", L1.bias), ("W2", L2.weight), ("B2", L2.bias),
          ("WMU", MU.weight), ("BMU", MU.bias)]
n_in, n_h1, n_h2 = L1.in_features, L1.out_features, L2.out_features


def array_c(nome, t):
    """Tensore -> array C 'static const float' piatto, in ordine riga per riga.
    Per i pesi (out, in): l'elemento [r][c] sta in posizione r*in + c."""
    v = t.detach().numpy().astype(np.float32).ravel()
    righe = [", ".join(f"{x:.9e}f" for x in v[i:i + 6]) for i in range(0, len(v), 6)]
    return f"static const float {nome}[{len(v)}] = {{\n  " + ",\n  ".join(righe) + "\n};\n"


with open("actor_weights.h", "w") as f:
    f.write("/* Generato da esporta_actor.py — NON modificare a mano */\n")
    f.write("#ifndef ACTOR_WEIGHTS_H\n#define ACTOR_WEIGHTS_H\n\n")
    f.write(f"#define ACT_N_IN  {n_in}\n#define ACT_N_H1  {n_h1}\n#define ACT_N_H2  {n_h2}\n\n")
    for nome, t in strati:
        f.write(array_c(nome, t) + "\n")
    f.write("#endif\n")

n_par = sum(t.numel() for _, t in strati)
print(f"actor_weights.h: {n_par} parametri = {n_par*4/1024:.1f} KB in flash")

# --- 2. Vettori di test: osservazioni realistiche nel range delle osservazioni ---
rng = np.random.default_rng(0)
lo, hi = model.observation_space.low, model.observation_space.high
obs = rng.uniform(np.maximum(lo, -1.5), np.minimum(hi, 1.5), size=(100, n_in)).astype(np.float32)
azioni, _ = model.predict(obs, deterministic=True)   # stessa funzione usata sul banco
azioni = azioni.astype(np.float32).ravel()

with open("test_vectors.h", "w") as f:
    f.write("/* Generato da esporta_actor.py */\n#ifndef TEST_VECTORS_H\n#define TEST_VECTORS_H\n\n")
    f.write(f"#define N_TEST {len(obs)}\n\n")
    f.write(array_c("TEST_OBS", torch.from_numpy(obs)) + "\n")
    f.write(array_c("TEST_AZIONE", torch.from_numpy(azioni)) + "\n#endif\n")
print(f"test_vectors.h: {len(obs)} osservazioni, azioni in [{azioni.min():.3f}, {azioni.max():.3f}]")
