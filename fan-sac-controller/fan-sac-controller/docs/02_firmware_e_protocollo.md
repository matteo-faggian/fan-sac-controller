# 2. Firmware e protocollo PC ↔ Arduino

File: `firmware/banco_sysid_v2/banco_sysid_v2.ino`. Lo stesso firmware serve sia per la
system identification sia per le prove della policy sul banco.

## Ruolo dell'Arduino

L'Arduino **non decide niente**: fa da ponte.

1. Riceve dal PC il comando $u$ e lo applica all'ESC.
2. A ogni nuovo campione dell'HX711 (~100 Hz) manda al PC una riga:
   ```
   t_us,raw,u_us,n_scartati
   ```
   `t_us` = timestamp Arduino (µs), `raw` = valore grezzo HX711, `u_us` = impulso
   effettivamente applicato, `n_scartati` = contatore dei comandi rifiutati.

Tutta la logica (test, policy) sta sul PC: per cambiare un test non si riprogramma la scheda.

## Formato del comando

Formato **fisso** di 6 caratteri più a capo:

```
U d d d d c \n        es.  U10001  ->  u = 10.00
```

- `dddd` = $u \cdot 100$ (da 0000 a 2500)
- `c` = cifra di controllo = (somma delle 4 cifre) mod 10

L'Arduino scarta qualsiasi riga con lunghezza o cifra di controllo sbagliate, e resta applicato
il comando precedente.

**Perché così:** nella prima versione il comando era testo libero (`U 10.0000`). La libreria
HX711 disabilitava gli interrupt durante la lettura e la UART perdeva caratteri:
`U 10.0000` diventava `U 0.0000` (stop) oppure `U 100000` (**piena potenza**), circa ogni 2 s.
Con il formato fisso un carattere perso cambia la lunghezza e la riga viene scartata.

## Lettura dell'HX711 senza bloccare gli interrupt

La lettura è scritta a mano (24 impulsi di clock + 1 per scegliere canale A, guadagno 128) e
**non** disabilita gli interrupt, così la UART non perde più byte. L'unico vincolo del chip è
che SCK non resti alto più di 60 µs (altrimenti va in power-down): gli interrupt
dell'Arduino durano pochi µs, quindi il margine è ampio.

Un valore `raw = -1` (tutti i bit a 1) significa linea DT sempre alta, cioè HX711 non
alimentato o filo staccato. Il PC scarta questi campioni.

## Watchdog

Se per 500 ms non arriva un comando **valido**, l'impulso torna al neutro. Il PC manda il
comando ogni 20 ms (controllo) o 50 ms (sysid): margine di 10–25 volte.

## Latenza della catena

| Passaggio | Tempo tipico |
|---|---|
| Conversione HX711 | fino a 10 ms |
| Seriale + USB + Windows | ~5–10 ms |
| Attesa del periodo dell'impulso ESC (50 Hz) | 0–20 ms |
| Risposta ESC + motore (ritardo puro misurato) | ~27 ms |

In anello chiuso, sul banco, il comportamento corrisponde a un ritardo di ~3 passi da 20 ms
(~60 ms): vedi l'esperimento F in [04](04_ambiente_e_training.md).
