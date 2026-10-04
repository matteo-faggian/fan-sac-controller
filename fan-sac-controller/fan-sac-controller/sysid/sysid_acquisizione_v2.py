# =============================================================================
# sysid_acquisizione_v2.py — esegue i test di identificazione sul banco e salva i dati
# -----------------------------------------------------------------------------
# Lavora insieme allo sketch banco_sysid_v2.ino caricato sull'Arduino.
# Novita' v2: comando a formato fisso con cifra di controllo, colonna n_scartati,
# programma fino a u=20 (sopra l'alimentatore va in limitazione), fase finale
# a motore fermo per misurare lo spostamento dello zero.
# Il PC decide QUALE comando dare e QUANDO; l'Arduino lo applica e rimanda le misure.
#
# Fasi (nell'ordine):
#   1. cal_zero     : 10 s senza peso, motore fermo  -> offset della load cell
#   2. cal_peso     : 10 s con un peso noto           -> fattore di scala conteggi -> N
#   3. rumore_fermo : 30 s motore fermo               -> rumore elettrico dell'HX711
#   4. statica_su/giu: scalini lenti 0 -> 0.40 -> 0   -> mappa statica, deadband, isteresi
#   5. gradino      : salti tra due livelli, ripetuti -> costante di tempo tau
#
# Requisiti: pip install pyserial
# Uso:       python sysid_acquisizione.py
# =============================================================================

import sys    # per uscire con un messaggio d'errore
import time   # per misurare il tempo sul PC
import csv    # per scrivere il file di dati

import serial  # libreria pyserial: comunicazione con la porta seriale dell'Arduino

# ---------------------------- Configurazione ---------------------------------
PORTA = "COM3"        # porta dell'Arduino: la trovi in Arduino IDE -> Strumenti -> Porta
BAUD = 115200         # deve coincidere con Serial.begin() dello sketch
U_MAX = 25.0          # limite di sicurezza (scala 0..40; l'Arduino lo applica comunque anche lui)
FILE_OUT = "sysid_dati.csv"  # file dove salvo tutto
T_KEEPALIVE = 0.05    # invio il comando ogni 50 ms: tiene vivo il watchdog (500 ms) dell'Arduino


def costruisci_programma():
    """Restituisce la lista delle fasi motore: (nome_fase, comando_u, durata_s)."""
    prog = []

    # --- Rumore a motore fermo ---
    prog.append(("rumore_fermo", 0.0, 30.0))

    # --- Mappa statica: passi fitti in basso (la curva e' piu' ripida li'), poi piu' larghi ---
    # Massimo 20: nel primo test sopra 20 l'alimentatore andava in limitazione.
    livelli = [0, 1, 2, 3, 4, 6, 8, 10, 12, 15, 18, 20]
    for u in livelli:                 # salita
        prog.append(("statica_su", u, 4.0))   # 4 s: ~25 tau, il transitorio e' esaurito
    for u in reversed(livelli):       # discesa: confrontandola con la salita vedo l'isteresi
        prog.append(("statica_giu", u, 4.0))

    # --- Gradini: coppie (basso, alto) a punti di lavoro diversi ---
    # Ripeto ogni salto 3 volte: mediando le ripetizioni riduco l'effetto del rumore.
    coppie = [(4, 10), (8, 15), (12, 20)]
    for basso, alto in coppie:
        prog.append(("gradino", basso, 3.0))      # mi porto e mi stabilizzo sul livello basso
        for _ in range(4):
            prog.append(("gradino", alto, 2.0))   # gradino in salita (2 s >> tau ~ 0.1 s)
            prog.append(("gradino", basso, 2.0))  # gradino in discesa

    # --- Partenze da fermo: il ritardo di avvio del motore e' molto piu' lungo ---
    for _ in range(4):
        prog.append(("partenza", 0, 3.0))         # motore fermo
        prog.append(("partenza", 10, 3.0))        # avvio a u=10

    # --- Motore fermo finale: misura dello spostamento dello zero dopo i test ---
    prog.append(("zero_fine", 0.0, 15.0))
    return prog


def invia_comando(ser, u):
    """Manda il comando u (scala 0..40) nel formato fisso 'Udddd c'.

    dddd = u*100 su 4 cifre, c = (somma delle 4 cifre) mod 10.
    Esempio: u = 10 -> 1000 -> somma 1 -> riga 'U10001'.
    L'Arduino scarta qualsiasi riga che non rispetti esattamente questo formato.
    """
    u = min(max(u, 0.0), U_MAX)
    v = int(round(u * 100))                      # 0..2500
    cifre = f"{v:04d}"                           # sempre 4 cifre, es. '0450'
    controllo = sum(int(c) for c in cifre) % 10  # cifra di controllo
    ser.write(f"U{cifre}{controllo}\n".encode("ascii"))

class LettoreRighe:
    """Raccoglie i byte dalla seriale e restituisce solo righe COMPLETE.

    La seriale consegna i dati a pezzi: una lettura puo' contenere mezza riga.
    Qui accumulo i byte in un buffer e restituisco le righe solo quando
    e' arrivato il carattere di fine riga '\\n'.
    """

    def __init__(self, ser):
        self.ser = ser
        self.buf = b""   # byte arrivati ma non ancora formanti una riga completa
        self.scarta_prima = False  # dopo uno svuotamento la prima riga puo' essere monca

    def righe(self):
        n = self.ser.in_waiting               # quanti byte sono gia' arrivati
        if n:
            self.buf += self.ser.read(n)      # li prendo tutti, senza aspettare
        *complete, self.buf = self.buf.split(b"\n")  # l'ultimo pezzo puo' essere incompleto
        if self.scarta_prima and complete:
            complete = complete[1:]           # butto la prima riga: puo' essere solo la sua coda
            self.scarta_prima = False
        return [r.decode("ascii", errors="ignore").strip() for r in complete]


def esegui_fase(ser, lettore, writer, nome, u, durata, massa_g="", svuota=False):
    """Applica il comando u per 'durata' secondi e salva tutti i campioni ricevuti.

    svuota=True: prima di iniziare butta via i dati arrivati in precedenza.
    Serve dopo un'attesa (es. input() dell'utente): l'Arduino continua a trasmettere
    e quelle righe vecchie finirebbero, sbagliando, dentro questa fase.
    Tra fasi consecutive del programma motore NON si svuota, per non perdere
    i campioni a cavallo dei gradini.

    Restituisce (numero_campioni, frequenza_Hz misurata dai timestamp dell'Arduino).
    """
    if svuota:
        ser.reset_input_buffer()   # scarto i byte gia' arrivati nel buffer di Windows
        lettore.buf = b""          # e anche l'eventuale mezza riga nel mio buffer
        lettore.scarta_prima = True  # il primo pezzo che arriva puo' iniziare a meta' riga
    t_inizio = time.time()
    t_ultimo_invio = 0.0
    n_campioni = 0
    t_us_primo = None   # timestamp Arduino del primo campione della fase
    t_us_ultimo = None  # timestamp Arduino dell'ultimo campione della fase
    print(f"  {nome:13s} u={u:.2f} per {durata:.0f} s")

    while time.time() - t_inizio < durata:
        ora = time.time()
        if ora - t_ultimo_invio >= T_KEEPALIVE:   # rimando il comando periodicamente
            invia_comando(ser, u)
            t_ultimo_invio = ora

        for riga in lettore.righe():              # tutte le righe arrivate nel frattempo
            parti = riga.split(",")
            if len(parti) != 4:                   # scarto righe malformate (es. "READY")
                continue
            try:
                t_us, raw, u_us, n_scart = (int(p) for p in parti)
            except ValueError:
                continue
            # una riga del CSV per ogni campione dell'HX711
            writer.writerow([nome, f"{u:.4f}", massa_g, t_us, raw, u_us, n_scart, f"{ora:.4f}"])
            esegui_fase.n_scartati = n_scart      # ultimo valore del contatore dell'Arduino
            n_campioni += 1
            if t_us_primo is None:
                t_us_primo = t_us                 # primo campione della fase
            t_us_ultimo = t_us                    # aggiorno l'ultimo

        time.sleep(0.002)                         # non saturo la CPU del PC

    # Frequenza = (campioni - 1) intervalli / tempo trascorso secondo l'orologio dell'Arduino.
    # Non dipende da ritardi della seriale o del PC.
    freq = 0.0
    if n_campioni > 1 and t_us_ultimo > t_us_primo:
        freq = (n_campioni - 1) / ((t_us_ultimo - t_us_primo) / 1e6)
    return n_campioni, freq


esegui_fase.n_scartati = 0   # contatore diagnostico, aggiornato durante le fasi


def main():
    print(f"Apro {PORTA}...")
    ser = serial.Serial(PORTA, BAUD, timeout=0)  # timeout=0: le letture non bloccano mai
    lettore = LettoreRighe(ser)

    # Aprendo la porta l'Arduino si resetta: aspetto il suo "READY" (arrivo ESC armato)
    print("Attendo READY dall'Arduino (circa 5 s)...")
    t0 = time.time()
    pronto = False
    while time.time() - t0 < 15:
        if "READY" in lettore.righe():
            pronto = True
            break
        time.sleep(0.01)
    if not pronto:
        sys.exit("Nessun READY: controlla porta, baud e che lo sketch sia caricato.")

    with open(FILE_OUT, "w", newline="") as f:
        writer = csv.writer(f)
        # intestazione: fase, comando richiesto, massa di calibrazione, tempo Arduino,
        # valore grezzo HX711, impulso effettivamente applicato, tempo PC
        writer.writerow(["fase", "u_cmd", "massa_g", "t_us", "raw", "u_us", "n_scartati", "t_pc"])

        try:
            # ---------------- Calibrazione (motore sempre fermo) ----------------
            input("\n[CAL] Togli ogni peso dalla load cell, poi premi Invio...")
            n, freq = esegui_fase(ser, lettore, writer, "cal_zero", 0.0, 10.0, svuota=True)

            # Controllo della frequenza dell'HX711: deve essere ~80 Hz, non ~10 Hz
            print(f"  -> frequenza HX711 misurata: {freq:.1f} Hz ({n} campioni)")
            if freq < 50:
                print("  ATTENZIONE: HX711 a ~10 Hz. Porta il pin RATE a VCC prima dei test dinamici.")
                if input("  Continuare comunque? [s/N] ").lower() != "s":
                    return

            massa = input("[CAL] Applica un peso noto nella direzione della misura e scrivi la massa in grammi: ")
            esegui_fase(ser, lettore, writer, "cal_peso", 0.0, 10.0,
                        massa_g=massa.strip(), svuota=True)

            input("[CAL] Togli il peso. Ora i test col MOTORE.\n"
                  "      Elica montata e serrata, banco bloccato, mani lontane. Invio per partire...")

            # ---------------- Test con il motore ----------------
            prog = costruisci_programma()
            durata_tot = sum(d for _, _, d in prog)
            print(f"\nInizio test motore: {len(prog)} fasi, circa {durata_tot/60:.1f} minuti")
            for i, (nome, u, durata) in enumerate(prog):
                # svuoto solo prima della prima fase (dopo l'attesa dell'Invio)
                esegui_fase(ser, lettore, writer, nome, u, durata, svuota=(i == 0))

            print(f"\nFatto. Dati salvati in {FILE_OUT}")
            print(f"Comandi scartati dall'Arduino (controllo fallito): {esegui_fase.n_scartati}")

        except KeyboardInterrupt:
            print("\nInterrotto con Ctrl+C.")

        finally:
            # In OGNI caso (fine normale, errore, Ctrl+C) porto il motore a zero
            for _ in range(5):
                invia_comando(ser, 0.0)
                time.sleep(0.02)
            ser.close()


if __name__ == "__main__":
    main()
