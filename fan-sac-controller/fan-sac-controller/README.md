# FAN SAC Controller

Controllo in forza di un fan (motore brushless + ESC) con **Reinforcement Learning (SAC)**,
addestrato in simulazione e trasferito su un banco prova reale.

Progetto didattico: lo scopo è capire come funziona il RL "dall'inizio alla fine", quindi il
controllo è **RL puro**, senza PID, nemmeno ibrido.

![Policy E sul banco reale](docs/img/prova_reale_E.png)

## Il problema

Un carrellino su una corsia inclinabile spinge, tramite il fan, contro una **load cell**.
La cella misura

$$F_{lc} = T - m\,g\,\sin\theta + \text{bias} + \text{rumore}$$

L'obiettivo è tenere $F_{lc} = 1.15$ N mentre l'inclinazione $\theta$ della corsia cambia
(entro ±14°). La rete **non vede** l'inclinazione: deve dedurla dall'effetto sulla forza.

## Pipeline

```
banco reale ──[test]──▶ dati ──[system identification]──▶ parametri del modello
                                                                  │
                                                                  ▼
          policy SAC (rete neurale) ◀──[training]── simulatore (env_fan.py)
                  │
                  ▼
     test sul banco reale (PC + Arduino)  ──▶  deploy su STM32 (in programma)
```

## Risultati principali

| | Simulatore | Banco reale |
|---|---|---|
| RMS errore di forza, policy E | 0.078–0.090 N | **0.067–0.085 N** |
| Errore medio, policy E | — | +0.003 N |
| RMS errore, policy F, ritardo 3 passi (come il banco) | 0.111 N (E: 0.151 N) | *prova in corso* |

Il trasferimento simulatore → banco ha funzionato: le prestazioni reali stanno nell'intervallo
previsto dalla simulazione. Sul banco la policy E mostrava un'oscillazione a ~3 Hz, dovuta al
ritardo dell'anello PC + Arduino (~60 ms). La policy F, addestrata con ritardi fino a 80 ms e
con la storia dei comandi nell'osservazione, in simulazione riduce quell'oscillazione di circa
tre volte (vedi [docs/04](docs/04_ambiente_e_training.md)).

**Stato:** sysid ✅ · simulatore validato ✅ · policy E sul banco ✅ · policy F in simulazione ✅ ·
policy F sul banco ⏳ · deploy su STM32 ⏳

## Struttura

| Cartella | Contenuto |
|---|---|
| `firmware/` | sketch Arduino: ponte PC ↔ ESC + HX711 |
| `sysid/` | acquisizione dati e analisi per la system identification |
| `controller/` | ambiente Gymnasium, training, valutazione, controllo sul banco reale |
| `data/` | dati grezzi: test sysid e prove reali |
| `models/` | policy addestrate |
| `docs/` | manuali |

## Manuali

1. [Banco e hardware](docs/01_banco_hardware.md): componenti, cablaggio, modifiche, calibrazione
2. [Firmware e protocollo](docs/02_firmware_e_protocollo.md): come PC e Arduino si parlano
3. [System identification](docs/03_system_identification.md): dai dati al modello del banco
4. [Ambiente e training](docs/04_ambiente_e_training.md): osservazioni, azioni, reward, esperimenti A–F
5. [Prove sul banco reale](docs/05_prove_sul_banco.md): come lanciare una prova in sicurezza
6. [Lezioni imparate](docs/06_lezioni_imparate.md): problemi incontrati, diagnosi e soluzioni

## Avvio rapido

```bash
pip install -r requirements.txt
cd controller
python env_fan.py          # training (300k step, ~1.5 h su CPU)
python valuta.py           # valutazione in simulazione del modello migliore
python controllo_reale.py  # prova sul banco (Arduino con firmware/banco_sysid_v2)
```

Autore: Matteo Faggian
