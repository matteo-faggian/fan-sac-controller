# 7. Deploy della policy sullo STM32

> Versione estesa con derivazioni e codice commentato: [appunti in PDF](appunti/appunti_deploy_stm32.pdf). Dettaglio sul clock: [Clock_Configuration_STM32F407.pdf](Clock_Configuration_STM32F407.pdf).

Obiettivo: far girare la policy **F** direttamente su un microcontrollore, senza PC
nell'anello di controllo. Questo manuale cresce passo per passo: ogni sezione descrive un
pezzo già verificato.

**Stato:** scelta della scheda ✅ · rete in C verificata sul PC ✅ · clock a 168 MHz ✅ ·
USB (DFU + seriale CDC) ✅ · test della rete sul chip ✅ · PWM ESC ✅ · HX711 ✅ ·
anello chiuso ✅ · prova sul banco ✅ (report: [08](08_prove_stm32.md))

## 7.1 Perché lo STM32F407 (e non il BluePill)

L'actor della policy è una rete 7 → 256 → 256 → 1. Il numero di parametri è

$$N = (7\cdot256 + 256) + (256\cdot256 + 256) + (256 + 1) = 68\,097$$

cioè $68\,097 \times 4\ \text{byte} \approx 266$ KB in float32.

| | BluePill (F103C8T6) | Black board (F407VET6) |
|---|---|---|
| Core | Cortex-M3, 72 MHz | Cortex-M4F, 168 MHz |
| FPU | ❌ (float emulato in software) | ✅ singola precisione |
| Flash / RAM | 64 KB / 20 KB | 512 KB / 192 KB |

Sul BluePill la rete non sta in flash e, senza FPU, ogni moltiplicazione float costa decine
di cicli: servirebbero quantizzazione e una rete più piccola. Sul F407 la rete entra così com'è.

Attenzione nella scelta: le famiglie **F0, G0, L0** (Cortex-M0/M0+) non hanno FPU, anche
quando hanno flash sufficiente. Servono le famiglie **F4, G4, L4** (Cortex-M4F).

## 7.2 Cosa deve fare il firmware

Lo STM32 sostituisce sia l'Arduino sia il PC:

```
timer 50 Hz ─▶ leggi HX711 ─▶ costruisci osservazione ─▶ rete ─▶ comando ─▶ PWM all'ESC
                              (filtro, integrale,
                               ultimi 3 comandi)
```

Senza USB e Windows nell'anello, il ritardo dovrebbe scendere a 1–2 passi (era ~3 con
PC + Arduino). La policy F è stata addestrata con ritardi da 1 a 4 passi, quindi va bene
in tutti e due i casi; la policy E invece no.

## 7.3 La rete in C, scritta a mano

File in `stm32/rete/`.

Due strade possibili: **X-CUBE-AI** (tool di ST che genera il C da un modello ONNX) oppure
scrivere l'inferenza a mano. Per una rete di tre strati la seconda è più semplice e
trasparente: sono tre prodotti matrice-vettore.

In inferenza deterministica la policy SAC calcola

$$a = \tanh\Big(W_\mu\, \text{ReLU}\big(W_2\, \text{ReLU}(W_1 x + b_1) + b_2\big) + b_\mu\Big)$$

Il ramo `log_std` serve solo in training (per campionare azioni casuali) e non viene esportato.

| File | Ruolo |
|---|---|
| `esporta_actor.py` | legge il modello `.zip` e scrive `actor_weights.h` (pesi) e `test_vectors.h` (prove) |
| `actor.c`, `actor.h` | inferenza in C puro, ~30 righe, nessuna libreria esterna |
| `test_pc.c` | confronta il C con PyTorch sui 100 vettori di test |

I due `.h` sono **generati** (non sono nel repository): si ricreano con

```bash
cd stm32/rete
python esporta_actor.py ../../models/sac_F.zip
gcc -O2 -o test_pc test_pc.c actor.c -lm
./test_pc
```

Come sono organizzati i pesi: una matrice $W$ di dimensione (uscite × ingressi) è salvata
come array piatto riga per riga, quindi l'elemento $W_{rc}$ sta in posizione `r*n_in + c`.
È lo stesso ordine con cui PyTorch tiene in memoria `Linear.weight`. Gli array sono
`static const`: il compilatore li mette in **flash**, non in RAM. In RAM servono solo i due
vettori nascosti, $2 \times 256 \times 4 = 2$ KB.

**Verifica sul PC (modello F):**

```
actor_weights.h: 68097 parametri = 266.0 KB in flash
test 0: C = -0.999328   PyTorch = -0.999328
test 1: C = +0.973925   PyTorch = +0.973925
test 2: C = +0.443066   PyTorch = +0.443066
errore massimo su 100 test: 3.87e-07  -> OK
```

Un errore di $4\cdot10^{-7}$ è solo arrotondamento della precisione singola (circa 7 cifre
significative): il codice C e la rete addestrata calcolano la stessa funzione.

## 7.4 Clock a 168 MHz

Di default il micro parte dall'oscillatore RC interno (HSI, 16 MHz). Per la massima
velocità si usa il **quarzo esterno da 8 MHz** della scheda (HSE) e il **PLL**, che
moltiplica la frequenza in tre passaggi:

$$f_{in} = \frac{f_{HSE}}{M} = \frac{8}{8} = 1\ \text{MHz} \qquad\text{(deve stare tra 1 e 2 MHz)}$$

$$f_{VCO} = f_{in}\cdot N = 1 \cdot 336 = 336\ \text{MHz} \qquad\text{(deve stare tra 100 e 432 MHz)}$$

$$f_{SYS} = \frac{f_{VCO}}{P} = \frac{336}{2} = 168\ \text{MHz}, \qquad f_{USB} = \frac{f_{VCO}}{Q} = \frac{336}{7} = 48\ \text{MHz}$$

La USB richiede **esattamente 48 MHz**: è il vincolo che fissa la scelta di $f_{VCO}$ e $Q$.

I bus delle periferiche hanno limiti propri, da cui i prescaler:

| Bus | Prescaler | Frequenza | Limite | Timer collegati |
|---|---|---|---|---|
| AHB (core, memorie) | /1 | 168 MHz | 168 MHz | — |
| APB1 | /4 | 42 MHz | 42 MHz | 84 MHz (×2 automatico) |
| APB2 | /2 | 84 MHz | 84 MHz | 168 MHz |

Quando il prescaler di un bus è diverso da 1, i timer su quel bus ricevono il doppio della
frequenza del bus: per questo i timer di APB1 contano a 84 MHz. Servirà per il PWM dell'ESC.

### Impostazioni in CubeMX

1. **RCC → HSE:** `Crystal/Ceramic Resonator` (verificare che il quarzo sulla scheda sia
   marcato 8.000).
2. **Clock Configuration:** input 8 MHz, PLL Source Mux = HSE, M = 8, N = 336, P = 2,
   Q = 7, System Clock Mux = PLLCLK, AHB /1, APB1 /4, APB2 /2.

In alternativa si può scrivere `168` nella casella HCLK e premere Invio: CubeMX calcola
da solo i divisori.

### Errore incontrato: N = 136 invece di 336

Con N = 136 CubeMX mostrava tutto a **68 MHz** e la casella "48MHz clocks" in **rosso** con
19.43. I sintomi si spiegano rifacendo i conti con il valore sbagliato:

$$\frac{8}{8}\cdot 136 = 136, \qquad \frac{136}{2} = 68\ \text{MHz}, \qquad \frac{136}{7} = 19.43\ \text{MHz}$$

Lezione: quando un numero non torna, ripercorrere la catena dei divisori dall'ingresso
fino all'uscita; il primo valore diverso dall'atteso indica dove sta l'errore.

## 7.5 Una sola USB per caricare e per comunicare

Scelta: niente ST-Link e niente adattatore USB-seriale, solo il connettore micro USB della
scheda. La stessa porta fa due mestieri in due momenti diversi, decisi dal jumper **BT0**:

| Momento | BT0 | Chi gestisce la USB | Il PC vede |
|---|---|---|---|
| Caricare il firmware | 3.3V | bootloader di fabbrica (DFU) | "STM32 BOOTLOADER" |
| Usare il firmware | GND | il nostro programma (classe CDC) | una porta seriale `COMx` |

All'accensione il micro legge BT0: se è alto esegue il bootloader scritto in fabbrica in una
memoria di sola lettura, se è basso esegue il programma in flash. BT1 resta sempre a GND.

**Caricare:** BT0 su 3.3V → collegare la USB → STM32CubeProgrammer, porta `USB`, Connect →
Erasing & Programming → file `.elf` del progetto → Start Programming → BT0 su GND →
scollegare e ricollegare.

**Comunicare:** il firmware usa la classe **CDC** (*Communication Device Class*, "Virtual
Port Com"): il PC vede una porta seriale normale, senza driver aggiuntivi su Windows 10/11.
Con una porta virtuale il baud rate impostato sul PC non conta.

Prezzo di questa scelta: **niente debug passo-passo** (servirebbe un ST-Link). La verifica
si fa stampando sulla seriale.

### Impostazioni in CubeMX

- **Connectivity → USB_OTG_FS → Mode:** `Device_Only` (attiva PA11 = DM e PA12 = DP)
- **Middleware → USB_DEVICE → Class For FS IP:** `Communication Device Class (Virtual Port Com)`
- **SYS → Debug:** `Serial Wire` (PA13, PA14 riservati: utile se un giorno si usa un ST-Link)
- **PA6 → GPIO_Output:** LED D2, per vedere che il programma gira

## 7.6 Primo firmware: la rete sul chip

Il primo firmware ripete sul microcontrollore lo stesso test fatto sul PC: calcola le 100
azioni di `test_vectors.h`, le confronta con quelle di PyTorch e misura quanti cicli di
clock richiede un'inferenza, usando il contatore di cicli della CPU (**DWT->CYCCNT**, che
conta a 168 MHz). Il risultato viene stampato sulla seriale USB ogni 2 secondi:

```
clock 168 MHz | errore max ... e-9 | inferenza ... cicli = ... us | OK
```

File da aggiungere al progetto CubeIDE: `actor.c` in `Core/Src`; `actor.h`,
`actor_weights.h`, `test_vectors.h` in `Core/Inc`.

Progetto completo: `stm32/firmware/FanStm32/` (si apre in CubeIDE con File → Import →
STM32CubeMX/STM32CubeIDE Project). Contiene già una copia di `actor_weights.h` e
`test_vectors.h` generati dal modello F, così compila subito dopo il clone.

**Compilazione (verificata):** 0 errori, 0 warning.

```
   text    data     bss     dec     hex  filename
 308156     332   11252  319740   4e0fc  FanStm32.elf
```

Flash = text + data = 308 488 B (58.8% di 512 KB); RAM = data + bss = 11 584 B (8.8% di 128 KB).

### Problemi incontrati nella creazione del progetto

1. Progetto generato per **IAR** (`TargetToolchain=EWARM V8.50`): mancavano `.cproject`, startup e
   linker script. Soluzione: Project Manager → Toolchain/IDE → STM32CubeIDE.
2. Un `.project` generico, creato da un'importazione precedente, impediva a CubeMX di scrivere i file
   di CubeIDE. Soluzione: togliere il progetto dal workspace, cancellare `.project`, rigenerare.
3. Residui della prima generazione (CMSIS completa con DSP/NN/RTOS e cartella `EWARM`) causavano
   15 errori di compilazione in file mai toccati. Soluzione: spostarli fuori dal progetto.

### Risultati sul chip

```
clock 168 MHz | errore max 387 e-9 | inferenza 3100163 cicli = 18453 us | OK   (build -O0)
clock 168 MHz | errore max 268 e-9 | inferenza 702472 cicli = 4181 us | OK     (build -O2)
```

- L'errore massimo rispetto a PyTorch è dell'ordine dell'arrotondamento dei `float` a 32 bit:
  **la rete in C sul chip è corretta**. Cambia un poco tra le due build perché il compilatore
  ottimizzato esegue le somme in un altro ordine.
- Con `-O0` (default della configurazione Debug) l'inferenza occupava il 92% del passo da 20 ms:
  46 cicli per moltiplicazione-somma. Con `-O2` scende a 10.4 cicli, 4.18 ms.
  Impostazione: Properties → C/C++ Build → Settings → MCU/MPU GCC Compiler → Optimization →
  *Optimize more (-O2)*.

## 7.7 Firmware di controllo

Il firmware finale (`Core/Src/main.c`, commentato riga per riga) fa sullo STM32 esattamente
quello che `controller/controllo_reale.py` faceva sul PC. Tutto il codice aggiunto sta nelle
sezioni `USER CODE` (CubeMX non lo cancella se si rigenera) e configura timer e pin
**direttamente nei registri**: il modulo HAL dei timer non è abilitato nel progetto.

### Collegamenti

| Da | A | Note |
|---|---|---|
| ESC segnale | **PB6** | TIM4 canale 1, funzione alternativa AF2 |
| ESC massa (connettore segnale) | GND | indispensabile: riferimento del segnale |
| ESC rosso (BEC 5 V) | — | non collegato: la scheda è alimentata dalla USB |
| HX711 SCK | **PB12** | uscita push-pull |
| HX711 DT | **PB13** | ingresso con pull-up interno: se il filo si stacca DT resta alto e il controllo si ferma |
| HX711 VCC / GND | 5 V / GND | vedi nota sotto |

**Nota sulla tensione dell'HX711.** Il datasheet chiede che l'HX711 abbia la *stessa
alimentazione del microcontrollore*. Con l'Arduino (5 V) era così, con lo STM32 (3.3 V) no.
A 5 V funziona (lo STM32 pilota SCK con 3.3 V e l'HX711 li riconosce), ma non ho verificato
sul datasheet ST che PB13 sopporti i 5 V che arrivano da DT. Le alternative sono HX711 a
3.3 V con ricalibrazione (comando `K`), oppure una resistenza da 4.7 kΩ in serie su DT.

### PWM dell'ESC

- Clock del timer: APB1 = 42 MHz con prescaler /4 ≠ 1, quindi i timer ricevono 2 × 42 = **84 MHz**.
  Il firmware lo calcola dai registri (`tim4_clock_hz()`) invece di darlo per scontato.
- PSC = 84 − 1 → 1 tick = 1 µs; ARR = 20000 − 1 → periodo 20 ms (50 Hz, come la libreria Servo).
  Il messaggio di avvio stampa `TIM4 PSC 83`: è la verifica che la scala sia giusta.
- Mappatura **identica** allo sketch `banco_sysid_v2.ino`: µs = 1472 − round(u/40 × 928),
  con u limitato a [0, 25]. L'impulso **diminuisce** quando la spinta aumenta.

### Ciclo di controllo (50 Hz)

Stesso ordine di `controllo_reale.py` (righe 271–293), costanti copiate da `env_fan.py`:
forza F = (raw − raw_zero)/k → derivata filtrata (β = 0.2) → errore e = 1.15 − F →
integrale saturato a ±0.5 → 7 ingressi [F/2.5, e/1.15, dF/2.5, c₋₁, c₋₂, c₋₃, I/0.5] →
rete → c = 0.05 + 0.95·(a+1)/2 → u = 20c → impulso. La memoria dei comandi si aggiorna
**dopo** aver calcolato il nuovo, come `registra_comando()`.

### Sicurezza

- All'accensione: neutro (1472 µs) subito, autotest della rete, 3 s di attesa per armare l'ESC.
- **Watchdog indipendente (IWDG)**: se il programma si blocca per ~0.5 s il chip si resetta, e al
  riavvio l'ESC riceve di nuovo il neutro.
- Stop automatico se |F| > 8 N, se non arriva un campione valido da più di 100 ms, dopo 180 s.
- La telemetria USB non rallenta mai il controllo: se la USB è occupata la riga si scarta e si conta.

### Comandi (seriale USB) e uso

`Z` zero · `K` calibrazione con 317 g · `S` avvio · `X` stop · `P` stato · `H` aiuto.

```
cd stm32
python monitor_stm32.py COM4
```

Lo script guida zero, eventuale calibrazione e avvio, e salva un CSV con le colonne
`t_ms, F_mN, c_x1000, u_us, inferenza_us, eta_ms`. Analisi: `python analizza_prova.py file.csv`.

### Caricare il firmware

BT0 su 3.3V → USB → STM32CubeProgrammer, porta `USB`, Connect → Erasing & Programming →
`Debug/FanStm32.elf` → **Start Programming** → attendere *Download verified successfully* →
Disconnect → BT0 su GND → scollegare e ricollegare. Aprire il file in CubeProgrammer **non** lo scrive.

