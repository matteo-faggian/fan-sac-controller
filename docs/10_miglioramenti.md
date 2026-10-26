# 10. Cosa si potrebbe migliorare

Elenco dei punti aperti a fine progetto (9 ottobre 2026). Ogni voce dice **da quale misura o
documento nasce** e, quando la soluzione non è stata provata, lo dice: sono proposte, non
risultati.

## 1. Tensione di lavoro dell'ESC (12 V contro 16 V)

**Da dove nasce:** [08](08_prove_stm32.md), §8.2. A 16 V l'RMS è 0.096 N, a 12 V 0.148 N. A 16 V
compaiono raffiche a ~4.4 Hz; l'ipotesi è che il guadagno comando→spinta sia più alto di quello
visto in training (rapporto misurato 1.67 contro 1.78 atteso da (16/12)²). **Non verificata.**

**Cosa fare:**
- controllare che ESC, motore e alimentatore siano dichiarati per 16 V;
- verificare l'ipotesi in simulazione: allargare l'intervallo del guadagno di spinta
  (oggi ×0.85–1.15) fino a coprire il guadagno a 16 V e vedere se le raffiche spariscono;
- se si resta a 16 V: rifare la sysid a 16 V e riaddestrare.

## 2. Riferimento di forza variabile

**Da dove nasce:** [08](08_prove_stm32.md), §8.4. Cambiando solo `F_REF` l'RMS randomizzato in
simulazione passa da 0.109 N (1.15 N) a 0.133 N (1.00 N) e 0.180 N (0.80 N), con la forza che resta
sopra il riferimento. La rete ha sempre visto F/2.5 ed e/1.15 legati dallo stesso riferimento.

**Cosa fare (proposta):** addestrare con il riferimento estratto a caso a ogni episodio (o a
gradini dentro l'episodio) e metterlo nell'osservazione. Va ricontrollato il dimensionamento:
per ogni riferimento F_ref ± m g sinθmax deve restare nella zona utile [0.4, 1.9] N.

## 3. Ritardo di training adatto allo STM32

**Da dove nasce:** la policy F è addestrata con ritardo 1–4 passi, pensato per PC + Arduino
(~3 passi). Sullo STM32 il ritardo residuo è meno di un passo (età del campione ≤ 11 ms +
inferenza 4.2 ms, [07](07_deploy_stm32.md)). Con 1–2 passi F è un po' peggiore di E
([04](04_ambiente_e_training.md), esperimento F).

**Cosa fare (proposta):** una policy dedicata allo STM32, con un intervallo di ritardo centrato
su quello misurato sul chip, tenendo margine (la randomizzazione deve contenere la realtà).
Da confrontare con F sullo stesso protocollo.

## 4. Confronti ripetibili tra le prove

**Da dove nasce:** le inclinazioni a mano non erano uguali nelle prove a 12 V e 16 V, quindi quel
confronto "non è una misura pulita" ([08](08_prove_stm32.md)). Manca anche il CSV della policy F
provata con PC + Arduino, quindi il confronto diretto E contro F sul banco non c'è.

**Cosa fare:**
- prova lunga con il protocollo del [05](05_prove_sul_banco.md) (20 s fermi, poi inclinazioni
  lente entro ±14°, stessa durata), per E, F su PC + Arduino e F sullo STM32, archiviando i CSV;
- (proposta) rendere ripetibili le inclinazioni con fermi meccanici ad angoli fissi. Un
  inclinometro usato **solo per registrare** θ nel CSV, senza darlo alla rete, permetterebbe di
  confrontare le prove senza cambiare il principio del progetto (la rete non vede θ).

## 5. Alimentazione dell'HX711 sullo STM32

**Da dove nasce:** [07](07_deploy_stm32.md). Il datasheet chiede la stessa alimentazione del
microcontrollore; l'HX711 è a 5 V e non è verificato che PB13 tolleri i 5 V su DT.

**Cosa fare:** HX711 a 3.3 V con ricalibrazione, oppure una resistenza da 4.7 kΩ in serie su DT.

## 6. Calibrazione e zero della cella

**Da dove nasce:** due calibrazioni hanno dato valori diversi del 17%
([01](01_banco_hardware.md)); dopo i test lo zero si sposta di ~+0.13 N
([03](03_system_identification.md)). Le cause indicate (punto di appoggio del peso, attrito o
isteresi meccanica) sono **ipotesi**.

**Cosa fare:**
- calibrare sempre con il peso nel punto in cui spinge il carrello, ripetere più volte e
  registrare la dispersione di k;
- misurare lo zero a motore fermo più volte nella stessa prova per capire se lo spostamento
  dipende dal tempo, dalla temperatura o dall'aver fatto girare il motore.

## 7. Rete più piccola

**Da dove nasce:** l'actor usa le dimensioni di default di Stable-Baselines3 (2 × 256, 68 097
parametri, 266 KB): sullo STM32 occupa il 57% della flash e 4.18 ms per passo
([07](07_deploy_stm32.md)). Non è stato provato se una rete più piccola basti.

**Cosa fare (proposta):** riaddestrare con 2 × 64 o 2 × 32 e confrontare RMS e offset lento in
simulazione con gli stessi seed. Se le prestazioni tengono, la rete starebbe anche su
microcontrollori più piccoli (con FPU).

## 8. Documentazione

- I numeri sull'inferenza "~1 ms" nei manuali 01 e 06 erano una stima precedente al deploy: il
  valore misurato è 4.18 ms con `-O2` (corretti in questa revisione).
- Il README e i manuali sono in italiano: un riassunto in inglese nel README renderebbe il
  progetto leggibile anche a chi arriva dal profilo LinkedIn.
