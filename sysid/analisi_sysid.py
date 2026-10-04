# =============================================================================
# analisi_sysid.py — ricava i parametri del banco da sysid_dati.csv (test v2)
# -----------------------------------------------------------------------------
# Output:
#   - stampa a video di tutti i parametri
#   - sysid_params.json : parametri pronti da caricare in env_fan.py
#   - analisi_sysid.png : grafici di controllo
# Uso: python analisi_sysid.py   (sysid_dati.csv nella stessa cartella)
# =============================================================================
import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.optimize import least_squares

FILE = "../data/sysid/sysid_dati_v2.csv"   # dati del test sysid v2 (cambia qui per un nuovo test)
G = 9.80665          # [m/s^2]
T_ASSEST = 1.5       # [s] nei livelli statici scarto il primo tratto (transitorio)
US_NEUTRO = 1472     # [us] impulso di motore fermo


def sigma_robusta(x):
    """Deviazione standard stimata dalla MAD: insensibile a pochi campioni anomali."""
    x = np.asarray(x)
    return 1.4826 * np.median(np.abs(x - np.median(x)))


def primo_ordine(p, t):
    """F(t) = F0 fino al ritardo L, poi esponenziale verso F1 con costante tau."""
    F0, F1, tau, L = p
    y = np.full_like(t, F0)
    i = t > L
    y[i] = F0 + (F1 - F0) * (1 - np.exp(-(t[i] - L) / tau))
    return y


# ---------------- Lettura e pulizia ----------------
d = pd.read_csv(FILE)
d = d[d.t_us.diff().fillna(1) > 0].reset_index(drop=True)   # solo timestamp crescenti
print(f"Comandi scartati dall'Arduino in tutto il test: {d.n_scartati.max()}")

# ---------------- Calibrazione ----------------
raw_zero = d.loc[d.fase == "cal_zero", "raw"].median()
cal = d[d.fase == "cal_peso"]
massa = float(cal.massa_g.dropna().iloc[0]) / 1000
k_cnt = (cal.raw.median() - raw_zero) / (massa * G)          # conteggi per Newton
d["F"] = (d.raw - raw_zero) / k_cnt
fs = 1e6 / d.t_us.diff().median()
print(f"Calibrazione: k = {k_cnt:.0f} conteggi/N   frequenza HX711 = {fs:.1f} Hz")

sig_fermo = sigma_robusta(d.loc[d.fase == "rumore_fermo", "F"])
print(f"Rumore a motore fermo: {sig_fermo*1000:.2f} mN")

m = d[~d.fase.isin(["cal_zero", "cal_peso", "rumore_fermo"])].reset_index(drop=True)
m["t"] = (m.t_us - m.t_us.iloc[0]) / 1e6
m["seg"] = ((m.fase != m.fase.shift()) | (m.u_cmd != m.u_cmd.shift())).cumsum()

# ---------------- Mappa statica + rumore a motore acceso ----------------
righe = []
for _, s in m[m.fase.str.startswith("statica")].groupby("seg"):
    s = s[s.t - s.t.iloc[0] > T_ASSEST]
    righe.append(dict(fase=s.fase.iloc[0], u=s.u_cmd.iloc[0], F=s.F.median(), sigma=sigma_robusta(s.F)))
mappa = pd.DataFrame(righe)
print("\nMappa statica [N]:")
print(mappa.pivot(index="u", columns="fase", values="F").round(3).to_string())

su = mappa[mappa.fase == "statica_su"].sort_values("u")
acc = mappa[mappa.F > 0.2]
A = np.c_[np.ones(len(acc)), acc.F]                   # rumore = s0 + s1*F (minimi quadrati)
s0, s1 = np.linalg.lstsq(A, acc.sigma, rcond=None)[0]
print(f"\nRumore a motore acceso: sigma = {s0*1000:.1f} mN + {s1*100:.1f}% * F")

zf = m[m.fase == "zero_fine"]
zero_fine = zf[zf.t - zf.t.iloc[0] > T_ASSEST].F.median()
print(f"Zero a motore fermo dopo i test: {zero_fine:+.3f} N")

# ---------------- Transitori ----------------
cambi = np.flatnonzero(m.u_us.diff().fillna(0).values != 0)
fit = []
for i in cambi:
    if m.fase[i] not in ("gradino", "partenza"):
        continue
    t0 = m.t[i]
    w = m[(m.t >= t0 - 0.05) & (m.t <= t0 + 1.5)]
    tt, yy = (w.t - t0).values, w.F.values
    r = least_squares(lambda p: primo_ordine(p, tt) - yy,
                      [yy[:4].mean(), np.median(yy[-50:]), 0.08, 0.04],
                      bounds=([-5, -5, 0.005, 0], [5, 5, 2, 0.8]))
    F0, F1, tau, L = r.x
    da, a = m.u_us[i - 1], m.u_us[i]
    tipo = ("avvio" if da == US_NEUTRO else "arresto" if a == US_NEUTRO
            else "salita" if F1 > F0 else "discesa")
    fit.append(dict(tipo=tipo, tau=tau, L=L))
fit = pd.DataFrame(fit)
stat = fit.groupby("tipo")[["tau", "L"]].median()
print("\nTransitori (mediane):")
print((stat * 1000).round(0).astype(int).rename(columns={"tau": "tau [ms]", "L": "ritardo [ms]"}).to_string())
in_moto = fit[fit.tipo.isin(["salita", "discesa"])]
print(f"tau in moto: da {in_moto.tau.min()*1000:.0f} a {in_moto.tau.max()*1000:.0f} ms")

# ---------------- Tabella "rotore in moto" e soglie di avvio/arresto ----------------
# La deadband ha isteresi: in salita il motore parte solo sopra una soglia, in discesa
# continua a girare anche sotto. Quindi servono DUE cose:
#   - soglia di avvio  : a meta' tra l'ultimo livello fermo e il primo in moto (salita)
#   - soglia di arresto: a meta' tra l'ultimo livello in moto e il primo fermo (discesa)
#   - tabella in moto  : media salita/discesa dove il motore gira in entrambe,
#                        piu' i livelli bassi dove gira solo in discesa.
# "In moto" = forza oltre 0.05 N (circa 15 sigma del rumore) sopra lo zero misurato a comando nullo nella stessa gamba.
giu = mappa[mappa.fase == "statica_giu"].sort_values("u")
z_su, z_giu = su.F.iloc[0], giu.F.iloc[0]
gira_su = su.F.values - z_su > 0.05
gira_giu = giu.F.values - z_giu > 0.05
u_vals = su.u.values
i_su = np.argmax(gira_su)                          # primo livello in moto in salita
u_avvio = 0.5 * (u_vals[i_su - 1] + u_vals[i_su])
i_giu = np.argmax(gira_giu)                        # primo livello in moto in discesa
u_arresto = 0.5 * (u_vals[i_giu - 1] + u_vals[i_giu]) if i_giu > 0 else 0.5 * u_vals[1]
F_moto = []
for j, u in enumerate(u_vals):
    if gira_su[j] and gira_giu[j]:
        F_moto.append(0.5 * ((su.F.values[j] - z_su) + (giu.F.values[j] - z_giu)))
    elif gira_giu[j]:
        F_moto.append(giu.F.values[j] - z_giu)
    else:
        F_moto.append(np.nan)
F_moto = np.array(F_moto)
ok = ~np.isnan(F_moto)
u_moto = u_vals[ok]
F_moto = np.maximum.accumulate(F_moto[ok])         # forza monotona: la saturazione non "scende"
print(f"\nSoglia di avvio: u = {u_avvio:.2f}   soglia di arresto: u = {u_arresto:.2f}")
print("Tabella in moto (zero sottratto):", dict(zip(u_moto.tolist(), F_moto.round(3).tolist())))

# ---------------- Parametri per l'ambiente ----------------
par = dict(
    u_tab_moto=u_moto.tolist(), F_tab_moto=F_moto.round(4).tolist(),
    u_avvio=float(u_avvio), u_arresto=float(u_arresto),
    u_tab=su.u.tolist(), F_tab=su.F.round(4).tolist(),   # mappa statica (salita)
    F_sat=float(su.F.max()),
    tau_moto=float(in_moto.tau.median()),
    tau_moto_min=float(in_moto.tau.min()), tau_moto_max=float(in_moto.tau.max()),
    tau_arresto=float(stat.loc["arresto", "tau"]),
    ritardo_avvio=float(stat.loc["avvio", "L"]),
    ritardo_moto=float(in_moto.L.median()),
    rumore_s0=float(s0), rumore_s1=float(s1),
    zero_dopo=float(zero_fine),
)
with open("sysid_params.json", "w") as f:
    json.dump(par, f, indent=2)
print("\nParametri salvati in sysid_params.json")

# ---------------- Grafici ----------------
fig, ax = plt.subplots(3, 1, figsize=(12, 11))
ax[0].plot(m.t, m.F, lw=0.5)
ax[0].set(xlabel="t [s]", ylabel="F [N]", title="Serie temporale (parte con motore)")
for fase, mk in [("statica_su", "o-"), ("statica_giu", "s--")]:
    q = mappa[mappa.fase == fase]
    ax[1].errorbar(q.u, q.F, yerr=q.sigma, fmt=mk, capsize=3, label=fase)
ax[1].set(xlabel="u (scala 0..40)", ylabel="F [N]", title="Mappa statica (barre = rumore 1 sigma)")
ax[1].grid(); ax[1].legend()
for i in cambi:
    if m.fase[i] != "gradino":
        continue
    t0 = m.t[i]
    w = m[(m.t >= t0 - 0.05) & (m.t <= t0 + 0.6)]
    y0, y1 = w.F.iloc[:4].mean(), w.F.iloc[-15:].median()
    if abs(y1 - y0) > 0.3:
        ax[2].plot(w.t - t0, (w.F - y0) / (y1 - y0), lw=0.6, alpha=0.6)
tt = np.linspace(0, 0.6, 300)
ax[2].plot(tt, primo_ordine([0, 1, par["tau_moto"], par["ritardo_moto"]], tt), "k", lw=2.5,
           label=f"primo ordine: tau={par['tau_moto']*1000:.0f} ms, L={par['ritardo_moto']*1000:.0f} ms")
ax[2].set(xlabel="t dal cambio di impulso [s]", ylabel="risposta normalizzata",
          title="Gradini a rotore in moto", ylim=(-0.4, 1.5))
ax[2].grid(); ax[2].legend()
plt.tight_layout(); plt.savefig("analisi_sysid.png", dpi=110)
print("Grafico salvato in analisi_sysid.png")
