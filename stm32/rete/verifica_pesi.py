# Confronta i pesi in actor_weights.h con quelli di ogni modello in models/
import re, sys, zipfile, io, numpy as np, torch
from pathlib import Path
qui = Path(__file__).parent
txt = (qui / "actor_weights.h").read_text(encoding="latin-1")
def arr(nome):
    m = re.search(nome + r"\[\d+\]\s*=\s*\{(.*?)\};", txt, re.S)
    return np.array([float(x.rstrip("fF")) for x in m.group(1).replace("\n", "").split(",") if x.strip()], dtype=np.float32)
W1h, WMUh = arr("W1"), arr("WMU")
for z in sorted((qui / "../../models").glob("*.zip")):
    with zipfile.ZipFile(z) as f:
        sd = torch.load(io.BytesIO(f.read("policy.pth")), map_location="cpu")
    w1 = sd["actor.latent_pi.0.weight"].numpy().ravel()
    wmu = sd["actor.mu.weight"].numpy().ravel()
    if w1.shape != W1h.shape:
        print(f"{z.name}: forma diversa (W1 {w1.size} pesi = {w1.size//256} ingressi, sul chip {W1h.size//256}) -> NON e' questo"); continue
    print(f"{z.name}: diff max W1 = {np.abs(w1 - W1h).max():.3e}, diff max WMU = {np.abs(wmu - WMUh).max():.3e}")
