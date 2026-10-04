# 5. Prove sul banco reale

File: `controller/controllo_reale.py`. La policy gira sul PC, l'Arduino fa da ponte.

## Preparazione

Nella cartella da cui lanci lo script servono:

- `controllo_reale.py`
- `env_fan.py` **della stessa versione con cui è stato addestrato il modello**
  (stesse osservazioni):
  - modello **F** (`models/sac_F.zip`, 7 osservazioni): `controller/env_fan.py` così com'è;
  - modello **E** (`models/sac_E.zip`, 5 osservazioni): `archivio/env_fan_E.py` rinominato
    in `env_fan.py`.

  Se le versioni non corrispondono, Stable-Baselines3 dà un errore sulla dimensione delle
  osservazioni.
- `sysid_params.json`
- il modello `.zip`

Sull'Arduino: `banco_sysid_v2.ino`. Monitor seriale dell'IDE chiuso. In cima allo script
controlla `PORTA` (es. `COM3`).

## Avvio

```bash
python controllo_reale.py                 # usa best_sac_fan/best_model.zip
python controllo_reale.py ../models/sac_F.zip   # un modello specifico
```

1. **Latenza:** 40 comandi sotto la soglia di avvio (il motore non parte); stampa mediana e
   massimo del tempo comando → conferma.
2. **Zero:** cella scarica, Invio. Opzionale: ricalibrazione con 317 g (`CALIBRA = True`).
3. **Grafico live:** forza, riferimento e comando, in un processo separato per non disturbare
   la temporizzazione.
4. **Controllo a 50 Hz:** temporizzato con `time.perf_counter()`, perché `time.sleep()` su
   Windows ha una risoluzione di ~15 ms. L'osservazione è costruita con le stesse funzioni di
   `env_fan.py`.
5. **Fine** con Ctrl+C o dopo 3 minuti: motore a zero e CSV `prova_reale_<data>_<ora>.csv`.

## Protezioni

| Condizione | Reazione |
|---|---|
| Campione HX711 non valido (`-1` o fondo scala) | scartato, si usa l'ultimo valido |
| Nessun campione valido da più di 100 ms | **stop** con messaggio |
| Forza oltre 8 N | **stop** |
| PC bloccato o USB scollegata | watchdog Arduino: neutro dopo 500 ms |

## Contenuto del CSV

`t`, `dt_ciclo` (durata reale del ciclo), `F`, `raw`, `eta_campione_ms` (età del campione
usato), `c`, `u`, `u_us_letto`, `azione`, `n_scartati`.

## Protocollo di prova consigliato

Per poter confrontare due policy:

1. 20 s fermi (misura dell'oscillazione "pura");
2. inclinazioni a mano, lente, entro ±14°, con ritmo e ampiezza simili tra le prove;
3. stessa durata.

## Risultati: policy E (ottobre 2026)

| Prova | Durata | RMS | Errore medio | Note |
|---|---|---|---|---|
| `E_guasto_sensore.csv` | 71 s | 0.085 N (primi 60 s) | +0.006 N | a 61 s l'HX711 smette di funzionare, vedi lezioni |
| `E_prova_breve.csv` | 21 s | 0.092 N | +0.003 N | oscillazione a ~3 Hz (47% della varianza) |
| `E_prova_175s.csv` | 175 s | **0.067 N** | +0.003 N | 0.022–0.030 N da fermo, picchi durante le inclinazioni |

![Prova reale E](img/prova_reale_E.png)

## Prossima prova: policy F

Stesso protocollo della prova E da 175 s (20 s fermi, poi inclinazioni a mano per ~3 minuti),
per confrontare direttamente le due policy. Attesa, dalla simulazione con ritardo 3: meno
oscillazione a ~3 Hz e comando più calmo.
