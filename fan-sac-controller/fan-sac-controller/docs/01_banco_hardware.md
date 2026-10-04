# 1. Banco e hardware

## Componenti

| Componente | Ruolo | Note |
|---|---|---|
| Fan brushless + elica | genera la spinta $T$ | |
| ESC (usato come **bidirezionale**) | pilota il motore da un segnale PWM tipo servo | neutro a metà corsa, vedi sotto |
| Load cell 1 kg + modulo HX711 | misura la forza del carrellino sulla cella | HX711 modificato a ~100 Hz |
| Corsia inclinabile + carrellino (317 g) | crea il disturbo gravitazionale $m g \sin\theta$ | inclinazione cambiata a mano |
| Arduino (Uno/Nano) | ponte PC ↔ ESC/HX711 | firmware `banco_sysid_v2.ino` |
| Alimentatore da banco | alimenta l'ESC | **limite di corrente impostato**: limita la spinta a ~2.2 N |
| STM32F407VET6 (black board) | futuro controllore stand-alone | ha FPU: esegue la rete da ~68k parametri in ~1 ms |

## Cablaggio (Arduino)

| Segnale | Pin Arduino |
|---|---|
| ESC segnale | D9 |
| ESC massa | GND (**massa comune obbligatoria**) |
| HX711 SCK | D2 |
| HX711 DT | D3 |
| HX711 VCC / GND | 5V / GND |

Il filo rosso (5 V del BEC) del connettore ESC resta **scollegato** quando l'Arduino è
alimentato via USB, per non mettere in parallelo due alimentazioni.

I fili HX711 ↔ Arduino vanno **saldati o fissati**: un contatto intermittente ha causato una
perdita completa della misura durante una prova (vedi [lezioni imparate](06_lezioni_imparate.md)).

## ESC bidirezionale: mappatura del comando

L'ESC ha lo **zero a metà corsa** (`Servo.write(90)`), non al minimo. Con `attach()` di
default la libreria Servo usa 544–2400 µs, quindi:

| Comando | Impulso | Effetto |
|---|---|---|
| neutro | 1472 µs | motore fermo |
| piena potenza nel verso che carica la cella | 544 µs | 100% |

Nel software il comando si esprime come $u \in [0, 40]$:

$$t_{imp} = 1472 - \frac{u}{40}\,(1472 - 544)\ [\mu s]$$

Limiti in uso: $u \le 20$ nella policy, $u \le 25$ come limite di sicurezza nel firmware.

## Modifica dell'HX711: da 10 a ~100 campioni/s

Il pin 15 dell'HX711 (RATE) sceglie la frequenza: a GND 10 Hz, a VCC 80 Hz nominali
(~100 Hz misurati su questo modulo). Sul modulo originale era saldato a GND. Modifica fatta:
pin 15 staccato da GND e collegato al pin 16 (DVDD).

Perché serve: la costante di tempo del rotore è ~78 ms. A 10 Hz si avrebbe meno di un campione
per costante di tempo e né la sysid né il controllo sarebbero possibili.

## Calibrazione della cella

Retta a due punti: zero (cella scarica) e massa nota (317 g):

$$F = \frac{raw - raw_0}{k}, \qquad k = \frac{raw_{peso} - raw_0}{m\,g}$$

Valore attuale: $k \approx -199\,950$ conteggi/N. Due calibrazioni successive hanno dato valori
diversi del 17%, probabilmente per il **punto** di appoggio del peso (le celle economiche sono
sensibili alla posizione del carico). Da rifare appoggiando la massa nel punto esatto in cui
spinge il carrellino. In training l'incertezza è coperta dalla randomizzazione del guadagno (±15%).

Lo **zero** si rimisura all'inizio di ogni prova (`controllo_reale.py` lo fa da solo).

## Sicurezza

- Elica sempre serrata, banco bloccato, mani lontane.
- Watchdog nel firmware: senza comandi validi per 500 ms il motore va al neutro.
- `controllo_reale.py` si ferma se la forza supera 8 N o se il sensore non dà campioni validi
  per più di 100 ms.
