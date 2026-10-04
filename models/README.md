# Modelli

| File | Esperimento | Ambiente da usare | Note |
|---|---|---|---|
| `sac_E.zip` | E: reward con \|e\|, γ = 0.99, i_max = 0.5, c_min = 0.05 | `controller/archivio/env_fan_E.py` | 5 osservazioni. Testato sul banco: RMS 0.067–0.085 N, oscillazione a ~3 Hz |
| `sac_F.zip` | F: E + ritardo 1–4 passi + ultimi 3 comandi osservati | `controller/env_fan.py` | 7 osservazioni. Miglior modello a 110k step. In simulazione con ritardo 3: RMS 0.111 N (E: 0.151 N). Da provare sul banco |

**Importante:** un modello funziona solo con l'ambiente con cui è stato addestrato, perché le
osservazioni devono avere lo stesso numero, lo stesso ordine e la stessa normalizzazione.
Per provare `sac_E.zip` sul banco, copia `archivio/env_fan_E.py` come `env_fan.py` nella
cartella da cui lanci `controllo_reale.py`. Per `sac_F.zip` va bene l'`env_fan.py` attuale.

Il modello si carica sempre con l'estensione esplicita:
`SAC.load("sac_E.zip")`. Senza `.zip`, se esiste una cartella con lo stesso nome,
Windows dà `PermissionError`.
