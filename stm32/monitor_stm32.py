# =============================================================================
# monitor_stm32.py — guida una prova con il controllo SULLA SCHEDA STM32 e salva i dati
# -----------------------------------------------------------------------------
# La rete gira sullo STM32 (firmware FanStm32). Il PC serve solo a:
#   1. mandare i comandi (Z = zero, S = start, X = stop)
#   2. ricevere la telemetria (una riga ogni 20 ms) e salvarla in un CSV
# Se il PC si scollega o questo script si chiude, la scheda continua da sola:
# per fermarla serve X, il reset, oppure le sicurezze interne (F > 8 N, sensore muto, 180 s).
#
# Uso:  python monitor_stm32.py            (porta COM4)
#       python monitor_stm32.py COM5
# =============================================================================
import sys
import time

import serial   # pyserial

PORTA = sys.argv[1] if len(sys.argv) > 1 else "COM4"


def leggi_righe(ser, secondi, stampa=True):
    """Legge per 'secondi' e restituisce le righe ricevute; stampa i messaggi '#'."""
    righe, t0 = [], time.time()
    while time.time() - t0 < secondi:
        r = ser.readline().decode("ascii", errors="ignore").strip()
        if r:
            righe.append(r)
            if stampa and r.startswith("#"):
                print(r)
    return righe


def main():
    ser = serial.Serial(PORTA, 115200, timeout=0.1)   # con la USB CDC il baud non conta
    ser.write(b"H\n"); leggi_righe(ser, 0.5)
    ser.write(b"P\n"); leggi_righe(ser, 0.5)

    input("\nCella SCARICA, motore fermo. Invio per lo ZERO...")
    ser.write(b"Z\n"); leggi_righe(ser, 4.0)

    if input("Vuoi ricalibrare con 317 g (serve se l'HX711 non e' piu' alimentato come in sysid)? [s/N] ").lower() == "s":
        input("Appoggia 317 g sulla cella, dove spinge il carrellino. Invio...")
        ser.write(b"K\n"); leggi_righe(ser, 4.0)
        input("Togli il peso. Invio...")

    input("\nElica montata, banco sicuro. Invio per AVVIARE il controllo (Ctrl+C per fermare)...")
    nome = time.strftime("prova_stm32_%Y%m%d_%H%M%S.csv")
    n = 0
    with open(nome, "w", newline="") as f:
        f.write("t_ms,F_mN,c_x1000,u_us,inferenza_us,eta_ms\n")
        ser.write(b"S\n")
        try:
            while True:
                r = ser.readline().decode("ascii", errors="ignore").strip()
                if not r:
                    continue
                if r.startswith("#"):
                    print("\n" + r)
                    if "STOP" in r:
                        break
                    continue
                if r.count(",") == 5:          # riga di telemetria completa
                    f.write(r + "\n"); n += 1
                    if n % 25 == 0:
                        t, F, c = r.split(",")[:3]
                        print(f"\rt = {int(t)/1000:6.1f} s   F = {int(F)/1000:5.2f} N   c = {int(c)/1000:.2f}   ", end="")
        except KeyboardInterrupt:
            print("\nCtrl+C: mando lo stop alla scheda")
            ser.write(b"X\n"); leggi_righe(ser, 0.5)
    ser.close()
    print(f"Salvate {n} righe in {nome}")


if __name__ == "__main__":
    main()
