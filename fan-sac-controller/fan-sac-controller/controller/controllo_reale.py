# =============================================================================
# controllo_reale.py — la policy SAC controlla il banco VERO, tramite Arduino
# -----------------------------------------------------------------------------
# Catena: HX711 -> Arduino (banco_sysid_v2.ino) -> USB -> PC: policy SAC -> USB -> Arduino -> ESC
#
# Cosa fa, in ordine:
#   1. misura la latenza USB PC <-> Arduino (motore fermo)
#   2. misura lo zero della cella (motore fermo, nessun carico)
#   3. ciclo di controllo a 50 Hz: legge la forza, costruisce l'osservazione ESATTAMENTE
#      come env_fan.py, chiede l'azione alla rete, manda il comando
#   4. grafico in tempo reale (forza, riferimento, comando) in una finestra separata
#   5. Ctrl+C (o fine tempo) -> motore a zero, salvataggio di tutto in un CSV
#
# File necessari nella stessa cartella: env_fan.py, sysid_params.json, il modello .zip
# Uso:   python controllo_reale.py                          (modello best_sac_fan/best_model.zip)
#        python controllo_reale.py percorso\modello.zip
# =============================================================================
import sys
import csv
import time
import multiprocessing as mp
from collections import deque
from pathlib import Path

import numpy as np

# ---------------------------- Configurazione ---------------------------------
PORTA = "COM3"
BAUD = 115200
DURATA_MAX = 180.0         # [s] durata massima della prova (poi si ferma da sola)
K_CNT_PER_N = -199948.5    # [conteggi/N] calibrazione del test sysid v2 (317 g sulla cella)
CALIBRA = False            # True: all'avvio chiede il peso noto e ricalcola K_CNT_PER_N
MASSA_CAL_G = 317.0        # [g] massa per la calibrazione (se CALIBRA = True)
F_ALLARME = 8.0            # [N] se la forza misurata supera questo valore -> stop immediato
ETA_MAX = 0.10             # [s] se l'ultimo campione VALIDO e' piu' vecchio di cosi' -> stop immediato
                           #     (sensore scollegato o bloccato: la policy volerebbe alla cieca)
GRAFICO = True             # finestra con la forza in tempo reale
FINESTRA_GRAFICO = 15.0    # [s] quanti secondi mostrare nel grafico


# =============================================================================
# Comunicazione con l'Arduino (stesso protocollo di sysid_acquisizione_v2.py)
# =============================================================================
def invia_u(ser, u):
    """Comando u (scala 0..40) nel formato fisso 'Udddd c' con cifra di controllo."""
    v = int(round(min(max(u, 0.0), 25.0) * 100))
    cifre = f"{v:04d}"
    ser.write(f"U{cifre}{sum(int(c) for c in cifre) % 10}\n".encode("ascii"))


class LettoreRighe:
    """Restituisce solo righe complete 't_us,raw,u_us,n_scartati' gia' convertite in interi."""

    def __init__(self, ser):
        self.ser = ser
        self.buf = b""

    def campioni(self):
        n = self.ser.in_waiting
        if n:
            self.buf += self.ser.read(n)
        *righe, self.buf = self.buf.split(b"\n")
        out = []
        for r in righe:
            p = r.decode("ascii", errors="ignore").strip().split(",")
            if len(p) == 4:
                try:
                    out.append(tuple(int(x) for x in p))
                except ValueError:
                    pass
        return out


def campione_valido(raw):
    """L'HX711 restituisce 24 bit in complemento a 2. Tre valori indicano un guasto:
    -1 (0xFFFFFF, tutti i bit a 1: linea DT sempre alta -> chip non alimentato o filo staccato)
    e i due estremi della scala (+8388607 / -8388608: ingresso saturato)."""
    return raw not in (-1, 8388607, -8388608)


def attendi(ser, lettore, secondi, u=0.0):
    """Tiene il comando u per 'secondi' (con keepalive) e restituisce i campioni ricevuti."""
    raccolti, t0, t_inv = [], time.perf_counter(), 0.0
    while time.perf_counter() - t0 < secondi:
        if time.perf_counter() - t_inv > 0.05:
            invia_u(ser, u)
            t_inv = time.perf_counter()
        raccolti += lettore.campioni()
        time.sleep(0.001)
    return raccolti


def misura_latenza(ser, lettore, n=40):
    """Tempo tra l'invio di un comando e il primo campione che riporta il nuovo impulso.

    Alterna u = 0 e u = 0.5: cambia l'impulso (1472 -> 1460 us) ma resta SOTTO la soglia
    di avvio del motore (~1.5), quindi il motore non parte.
    Il valore include l'attesa del prossimo campione HX711 (0..10 ms): e' un limite superiore.
    """
    lat = []
    for k in range(n):
        u = 0.5 if k % 2 == 0 else 0.0
        us_atteso = 1472 - int(u / 40 * (1472 - 544) + 0.5)
        lettore.campioni()                       # svuoto
        t0 = time.perf_counter()
        invia_u(ser, u)
        trovato = False
        while time.perf_counter() - t0 < 0.3 and not trovato:
            for (_, _, u_us, _) in lettore.campioni():
                if u_us == us_atteso:
                    lat.append(time.perf_counter() - t0)
                    trovato = True
                    break
        attendi(ser, lettore, 0.05, u)
    invia_u(ser, 0.0)
    return np.array(lat) * 1000


# =============================================================================
# Grafico in tempo reale: gira in un PROCESSO separato, cosi' il disegno (lento)
# non disturba la temporizzazione del ciclo di controllo.
# =============================================================================
def processo_grafico(coda, f_ref, finestra):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    T, F, C = deque(), deque(), deque()
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    lf, = ax1.plot([], [], lw=1, label="forza misurata")
    ax1.axhline(f_ref, color="k", ls="--", label=f"riferimento {f_ref:.2f} N")
    ax1.set_ylabel("F [N]"); ax1.set_ylim(-0.3, 2.6); ax1.grid(); ax1.legend(loc="upper left")
    lc, = ax2.plot([], [], lw=1, color="tab:orange")
    ax2.set_ylabel("comando c"); ax2.set_ylim(0, 1); ax2.set_xlabel("t [s]"); ax2.grid()
    testo = ax1.text(0.99, 0.95, "", transform=ax1.transAxes, ha="right", va="top",
                     fontsize=14, family="monospace")

    def aggiorna(_):
        while not coda.empty():
            msg = coda.get_nowait()
            if msg is None:
                plt.close(fig)
                return lf, lc, testo
            t, f, c = msg
            T.append(t); F.append(f); C.append(c)
        while T and T[-1] - T[0] > finestra:
            T.popleft(); F.popleft(); C.popleft()
        if T:
            lf.set_data(T, F); lc.set_data(T, C)
            ax1.set_xlim(max(0, T[-1] - finestra), max(finestra, T[-1]))
            testo.set_text(f"F = {F[-1]:5.2f} N")
        return lf, lc, testo

    _anim = FuncAnimation(fig, aggiorna, interval=100, cache_frame_data=False)
    plt.tight_layout()
    plt.show()


# =============================================================================
# Programma principale
# =============================================================================
def main():
    import serial
    from stable_baselines3 import SAC
    from env_fan import FanEnv

    percorso = sys.argv[1] if len(sys.argv) > 1 else "best_sac_fan/best_model.zip"
    model = SAC.load(percorso, device="cpu")
    # Uso l'ambiente solo come "libreria": stesse costanti e stessa funzione di osservazione
    # usate in training (f_scale, i_max, beta_df, c_min, u_max, force_ref, clip).
    env = FanEnv(domain_randomization=False)
    env.reset(seed=0)
    dt = env.dt
    print(f"Modello: {percorso}   riferimento: {env.force_ref} N   dt: {dt*1000:.0f} ms")

    ser = serial.Serial(PORTA, BAUD, timeout=0)
    lettore = LettoreRighe(ser)
    print("Attendo READY (ESC che si arma)...")
    t0, ricevuto = time.time(), b""
    while time.time() - t0 < 15:
        ricevuto += ser.read(ser.in_waiting or 1)      # accumulo: "READY" puo' arrivare a pezzi
        if b"READY" in ricevuto:
            break
        time.sleep(0.01)
    else:
        sys.exit("Nessun READY: controlla porta e sketch (serve banco_sysid_v2.ino).")
    attendi(ser, lettore, 0.5)

    # ---------------- 1. Latenza ----------------
    lat = misura_latenza(ser, lettore)
    print(f"Latenza comando->campione: mediana {np.median(lat):.1f} ms, max {lat.max():.1f} ms "
          f"(include fino a 10 ms di attesa del campione HX711)")

    # ---------------- 2. Zero (ed eventuale calibrazione) ----------------
    input("Cella SCARICA, motore fermo. Invio per misurare lo zero...")
    raw_zero = np.median([c[1] for c in attendi(ser, lettore, 3.0) if campione_valido(c[1])])
    k = K_CNT_PER_N
    if CALIBRA:
        input(f"Appoggia {MASSA_CAL_G:.0f} g sulla cella, nel punto dove spinge il carrellino. Invio...")
        raw_peso = np.median([c[1] for c in attendi(ser, lettore, 3.0) if campione_valido(c[1])])
        k = (raw_peso - raw_zero) / (MASSA_CAL_G / 1000 * 9.80665)
        print(f"Nuova calibrazione: k = {k:.0f} conteggi/N (sysid v2: {K_CNT_PER_N:.0f})")
        input("Togli il peso. Invio...")
        attendi(ser, lettore, 0.5)

    def forza(raw):
        return (raw - raw_zero) / k

    # ---------------- 3. Grafico ----------------
    coda = None
    if GRAFICO:
        coda = mp.Queue(maxsize=2000)
        p_graf = mp.Process(target=processo_grafico, args=(coda, env.force_ref, FINESTRA_GRAFICO),
                            daemon=True)
        p_graf.start()
        time.sleep(2.0)            # tempo per aprire la finestra

    input("\nElica montata, banco sicuro. Invio per avviare il CONTROLLO (Ctrl+C per fermare)...")

    # ---------------- 4. Ciclo di controllo ----------------
    nome_log = time.strftime("prova_reale_%Y%m%d_%H%M%S.csv")
    f_log = open(nome_log, "w", newline="")
    log = csv.writer(f_log)
    log.writerow(["t", "dt_ciclo", "F", "raw", "eta_campione_ms", "c", "u", "u_us_letto",
                  "azione", "n_scartati"])

    # stato identico a env.reset(): motore fermo, integrale, filtro e memoria dei comandi azzerati
    env.reset(seed=0)
    env.integral, env.df_filt = 0.0, 0.0
    # Uso solo campioni VALIDI: un campione -1 non deve mai arrivare alla rete.
    ultimo = None                    # ultimo campione VALIDO ricevuto
    n_invalidi = 0
    lettore.campioni()               # scarto il vecchio
    validi = [c for c in attendi(ser, lettore, 0.3) if campione_valido(c[1])]
    if not validi:
        sys.exit("Nessun campione valido dall'HX711: controlla i collegamenti della cella.")
    ultimo = validi[-1]
    t_ricezione = time.perf_counter()
    F_prec = forza(ultimo[1])

    def raccogli(istante):
        """Legge i campioni arrivati: aggiorna l'ultimo valido, conta quelli non validi."""
        nonlocal ultimo, t_ricezione, n_invalidi
        for s in lettore.campioni():
            if campione_valido(s[1]):
                ultimo, t_ricezione = s, istante
            else:
                n_invalidi += 1

    t_start = time.perf_counter()
    t_prossimo = t_start
    t_prec = t_start
    try:
        while True:
            # --- attesa precisa del prossimo istante (perf_counter, non time.sleep) ---
            t_prossimo += dt
            while time.perf_counter() < t_prossimo:
                raccogli(time.perf_counter())       # intanto raccolgo i campioni in arrivo
            ora = time.perf_counter()
            raccogli(ora)
            t = ora - t_start
            if t > DURATA_MAX:
                print("\nDurata massima raggiunta.")
                break

            # --- controllo del sensore: senza misure valide recenti, stop ---
            if ora - t_ricezione > ETA_MAX:
                print(f"\nSTOP: nessun campione valido dall'HX711 da {(ora - t_ricezione)*1000:.0f} ms "
                      f"(campioni non validi finora: {n_invalidi}). Controlla i fili della cella/HX711.")
                break

            # --- misura e osservazione, identiche a FanEnv.step() ---
            F = forza(ultimo[1])
            if abs(F) > F_ALLARME:
                print(f"\nALLARME: forza {F:.2f} N oltre {F_ALLARME} N. Stop.")
                break
            env.df_filt += env.beta_df * ((F - F_prec) - env.df_filt)
            errore = env.force_ref - F
            env.integral = float(np.clip(env.integral + errore * dt, -env.i_max, env.i_max))
            obs = env._get_obs(F, env.df_filt, errore)

            # --- azione della rete e comando ---
            a, _ = model.predict(obs, deterministic=True)
            a = float(np.clip(a[0], -1.0, 1.0))
            c = env.c_min + (1.0 - env.c_min) * 0.5 * (a + 1.0)
            u = c * env.u_max
            invia_u(ser, u)

            # memoria dei comandi: la versione F dell'ambiente ne usa 3 nell'osservazione
            if hasattr(env, "registra_comando"):
                env.registra_comando(c)
            else:
                env.last_c = c
            F_prec = F

            log.writerow([f"{t:.4f}", f"{(ora - t_prec)*1000:.2f}", f"{F:.4f}", ultimo[1],
                          f"{(ora - t_ricezione)*1000:.1f}", f"{c:.4f}", f"{u:.3f}", ultimo[2],
                          f"{a:.4f}", ultimo[3]])
            t_prec = ora
            if coda is not None and not coda.full():
                coda.put_nowait((t, F, c))
            if int(t / dt) % 25 == 0:
                print(f"\rt = {t:6.1f} s   F = {F:5.2f} N   c = {c:.2f}   ", end="")

    except KeyboardInterrupt:
        print("\nFermato con Ctrl+C.")
    finally:
        for _ in range(10):
            invia_u(ser, 0.0)
            time.sleep(0.02)
        ser.close()
        f_log.close()
        if coda is not None:
            coda.put(None)
        print(f"Motore a zero. Dati salvati in {nome_log}")
        print(f"Campioni HX711 non validi scartati: {n_invalidi}")
        print("Chiudi la finestra del grafico per terminare.")


if __name__ == "__main__":
    mp.freeze_support()     # necessario su Windows per il processo del grafico
    main()
