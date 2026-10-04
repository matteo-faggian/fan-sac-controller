// =============================================================================
// banco_sysid_v2.ino — Arduino come "ponte" tra PC e banco (ESC + load cell HX711)
// -----------------------------------------------------------------------------
// Novita' rispetto alla v1 (vedi analisi dei dati del primo test):
//
//  1. HX711 letto "a mano" SENZA disabilitare gli interrupt.
//     La libreria HX711 blocca gli interrupt durante la lettura dei 24 bit: in quel
//     tempo la UART perdeva i caratteri in arrivo dal PC e i comandi si corrompevano
//     ("U 10.0000" -> "U 0.0000" = stop, oppure "U 100000" = piena potenza).
//
//  2. Protocollo con controllo: il comando ha formato FISSO di 6 caratteri
//        U d d d d c      es. "U10001" -> u = 10.00, cifra di controllo 1
//     dddd = u * 100 (0000..4000), c = (somma delle 4 cifre) mod 10.
//     Se la lunghezza o la cifra di controllo non tornano, il comando viene SCARTATO
//     e resta applicato quello precedente. Un carattere perso non puo' piu'
//     trasformarsi in un comando diverso.
//
//  3. Watchdog (500 ms senza comandi validi -> neutro) e limite U_LIMITE ripristinati.
//
//  4. Stessa mappatura dello sketch che hai verificato: neutro = 1472 us,
//     piena potenza (nel verso che spinge la cella) = 544 us, u in scala 0..40.
//     Uso writeMicroseconds: risoluzione 1 us invece dei ~10 us di un grado di write().
//
// Uscita, una riga per campione HX711:   t_us,raw,u_us,n_scartati
//
// Cablaggio: ESC segnale -> D9, ESC GND -> GND Arduino
//            HX711 SCK -> D2, DT -> D3, VCC -> 5V, GND -> GND
// Libreria necessaria: solo Servo (inclusa nell'IDE). La libreria HX711 NON serve piu'.
// =============================================================================

#include <Servo.h>

// ---------------------------- Pin -------------------------------------------
const uint8_t PIN_ESC    = 9;
const uint8_t PIN_HX_SCK = 2;
const uint8_t PIN_HX_DT  = 3;

// ---------------------------- Mappatura ESC ---------------------------------
const int   US_NEUTRO = 1472;   // = write(90) con attach() di default: motore fermo
const int   US_PIENO  = 544;    // = write(0): piena potenza nel verso che carica la cella
const float U_SCALA   = 40.0;   // u = 40 corrisponde a US_PIENO (100% della corsa)
const float U_LIMITE  = 25.0;   // limite di sicurezza: oltre questo valore l'alimentatore
                                // va in limitazione (dati del primo test)

// ---------------------------- Sicurezza --------------------------------------
const unsigned long WATCHDOG_MS = 500;   // senza comandi validi per 500 ms -> neutro

// ---------------------------- Stato -----------------------------------------
Servo esc;
int u_us = US_NEUTRO;                // impulso attualmente applicato [us]
unsigned long t_ultimo_valido = 0;   // millis() dell'ultimo comando valido
unsigned int n_scartati = 0;         // comandi rifiutati dal controllo (diagnostica)

char buf[16];
uint8_t n_buf = 0;

// -----------------------------------------------------------------------------
// u (0..U_SCALA) -> larghezza d'impulso. Mappa lineare tra neutro e pieno,
// con saturazione al limite di sicurezza.
// -----------------------------------------------------------------------------
int u_to_us(float u) {
  if (u < 0.0)      u = 0.0;
  if (u > U_LIMITE) u = U_LIMITE;
  // u=0 -> 1472 us, u=40 -> 544 us: l'impulso DIMINUISCE quando la spinta aumenta
  return US_NEUTRO - (int)(u / U_SCALA * (US_NEUTRO - US_PIENO) + 0.5);
}

// -----------------------------------------------------------------------------
// Controlla e applica una riga ricevuta. Formato atteso ESATTO: "Udddd c" senza spazi.
// -----------------------------------------------------------------------------
void gestisci_riga() {
  // 1. lunghezza esatta: un carattere perso o in piu' -> scarto
  if (n_buf != 6 || buf[0] != 'U') { n_scartati++; return; }

  // 2. tutte cifre, e calcolo la somma delle prime 4
  int valore = 0, somma = 0;
  for (uint8_t i = 1; i <= 5; i++) {
    if (buf[i] < '0' || buf[i] > '9') { n_scartati++; return; }
  }
  for (uint8_t i = 1; i <= 4; i++) {
    int cifra = buf[i] - '0';
    valore = valore * 10 + cifra;   // costruisco il numero dddd
    somma += cifra;
  }

  // 3. cifra di controllo: deve essere uguale alla somma mod 10
  if ((somma % 10) != (buf[5] - '0')) { n_scartati++; return; }

  // 4. comando valido: lo applico
  u_us = u_to_us(valore / 100.0);
  esc.writeMicroseconds(u_us);
  t_ultimo_valido = millis();
}

// -----------------------------------------------------------------------------
// Lettura HX711 senza bloccare gli interrupt.
// Va chiamata solo quando DT e' BASSO (campione pronto).
// Protocollo del chip: 24 impulsi su SCK; a ogni impulso il chip presenta un bit
// su DT (MSB per primo). Un 25-esimo impulso sceglie canale A, guadagno 128.
// Vincolo: SCK non deve restare ALTO per piu' di 60 us, altrimenti il chip va in
// power-down. Gli interrupt dell'Arduino durano pochi us, quindi siamo sicuri.
// -----------------------------------------------------------------------------
long hx711_leggi() {
  unsigned long v = 0;
  for (uint8_t i = 0; i < 24; i++) {
    digitalWrite(PIN_HX_SCK, HIGH);       // fronte di salita: il chip mette il bit su DT
    delayMicroseconds(1);                 // attesa minima perche' il bit sia stabile
    v = (v << 1) | digitalRead(PIN_HX_DT);
    digitalWrite(PIN_HX_SCK, LOW);
    delayMicroseconds(1);
  }
  digitalWrite(PIN_HX_SCK, HIGH);         // 25-esimo impulso: canale A, guadagno 128
  delayMicroseconds(1);
  digitalWrite(PIN_HX_SCK, LOW);

  // il dato e' in complemento a 2 su 24 bit: estendo il segno a 32 bit
  if (v & 0x800000UL) v |= 0xFF000000UL;
  return (long)v;
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_HX_SCK, OUTPUT);
  digitalWrite(PIN_HX_SCK, LOW);          // SCK basso = HX711 attivo
  pinMode(PIN_HX_DT, INPUT);

  esc.attach(PIN_ESC);                    // limiti di default 544..2400 us, come lo sketch verificato
  esc.writeMicroseconds(US_NEUTRO);       // neutro: l'ESC si arma

  delay(3000);
  Serial.println(F("READY"));
  t_ultimo_valido = millis();
}

void loop() {
  // --- 1. Ricezione comandi ---
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      gestisci_riga();
      n_buf = 0;
    } else if (c != '\r') {
      if (n_buf < sizeof(buf) - 1) buf[n_buf++] = c;
      else n_buf = sizeof(buf);           // riga troppo lunga: verra' scartata
    }
  }

  // --- 2. Watchdog ---
  if (millis() - t_ultimo_valido > WATCHDOG_MS && u_us != US_NEUTRO) {
    u_us = US_NEUTRO;
    esc.writeMicroseconds(US_NEUTRO);
  }

  // --- 3. Campione HX711 ---
  if (digitalRead(PIN_HX_DT) == LOW) {    // DT basso = campione pronto
    unsigned long t = micros();
    long raw = hx711_leggi();
    Serial.print(t);          Serial.print(',');
    Serial.print(raw);        Serial.print(',');
    Serial.print(u_us);       Serial.print(',');
    Serial.println(n_scartati);
  }
}
