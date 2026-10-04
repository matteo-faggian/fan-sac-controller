# Dati

| File | Cosa contiene |
|---|---|
| `sysid/sysid_dati_v2.csv` | **test di system identification usato per il modello** (firmware v2, dati puliti) |
| `sysid/sysid_dati_v0_esc_fuori_scala.csv` | primo tentativo (1 ottobre 2026): impulsi 1000–1400 µs, sotto il neutro dell'ESC bidirezionale, il motore non gira. Archiviato come documentazione della lezione in [docs/06](../docs/06_lezioni_imparate.md) |
| `prove_reali/E_guasto_sensore.csv` | policy E sul banco: a 61 s l'HX711 smette di funzionare (`raw = -1`) |
| `prove_reali/E_prova_breve.csv` | policy E, 21 s, oscillazione a ~3 Hz |
| `prove_reali/E_prova_175s.csv` | policy E, 175 s, prova di riferimento per il confronto con F |

Formato delle prove reali: vedi [docs/05](../docs/05_prove_sul_banco.md).
