// =============================================================================
// banco_sysid.ino — Arduino come "ponte" tra PC e banco (ESC + load cell HX711)
// -----------------------------------------------------------------------------
// Ruolo: l'Arduino NON decide niente. Fa solo tre cose:
//   1. riceve dal PC il comando per l'ESC (riga di testo "U 0.250")
//   2. lo applica all'ESC come impulso PWM (1000..2000 us)
//   3. a ogni nuovo campione dell'HX711 manda al PC: tempo, valore grezzo, comando
// Tutta la logica dei test sta nello script Python: cosi' per cambiare un test
// non serve riprogrammare la scheda.
//
// Cablaggio:
//   ESC segnale -> D9      ESC GND -> GND Arduino (massa comune OBBLIGATORIA)
//   HX711 SCK   -> D2      HX711 DT -> D3      HX711 VCC -> 5V      GND -> GND
//
// Librerie: Servo (inclusa nell'IDE) e "HX711 Arduino Library" di Bogdan Necula
// (Arduino IDE -> Gestore librerie -> cerca "HX711").
// =============================================================================

#include <Servo.h>   // libreria per generare l'impulso stile servo che l'ESC capisce
#include "HX711.h"   // libreria per leggere il convertitore a 24 bit della load cell

// ---------------------------- Pin -------------------------------------------
const uint8_t PIN_ESC    = 9;  // uscita del segnale verso l'ESC
const uint8_t PIN_HX_SCK = 2;  // clock dell'HX711: l'Arduino lo pilota per leggere i bit
const uint8_t PIN_HX_DT  = 3;  // dati dell'HX711: va BASSO quando un campione e' pronto

// ---------------------------- Parametri ESC ----------------------------------
const int   US_MIN  = 1000;  // larghezza impulso [us] = motore fermo (comando 0)
const int   US_MAX  = 2000;  // larghezza impulso [us] = massima potenza (comando 1)
const float ESC_MAX = 0.40;  // limite di sicurezza: oltre 0.40 la spinta supera la portata
                             // della load cell da 1 kg (vedi dimensionamento del banco)

// ---------------------------- Sicurezza --------------------------------------
const unsigned long WATCHDOG_MS = 500;  // se il PC tace per piu' di 500 ms -> motore a zero
                                        // (protegge se lo script Python si blocca o crasha)

// ---------------------------- Oggetti e stato --------------------------------
Servo esc;      // oggetto che genera l'impulso su PIN_ESC usando un timer hardware
HX711 scale;    // oggetto che gestisce la comunicazione con l'HX711

int u_us = US_MIN;               // larghezza d'impulso attualmente applicata [us]
unsigned long t_last_cmd = 0;    // istante (millis) dell'ultimo comando ricevuto dal PC

char buf[32];       // buffer dove accumulo i caratteri in arrivo dalla seriale
uint8_t n_buf = 0;  // quanti caratteri ci sono nel buffer

// -----------------------------------------------------------------------------
// Converte un comando normalizzato u in [0, 1] nella larghezza d'impulso in us.
// Applica SEMPRE il limite ESC_MAX: anche se il PC sbaglia, l'Arduino non lo supera.
// -----------------------------------------------------------------------------
int u_to_us(float u) {
  if (u < 0.0)     u = 0.0;      // niente comandi negativi
  if (u > ESC_MAX) u = ESC_MAX;  // saturazione di sicurezza
  // mappa lineare: u=0 -> 1000 us, u=1 -> 2000 us; +0.5 per arrotondare all'intero
  return US_MIN + (int)(u * (US_MAX - US_MIN) + 0.5);
}

// -----------------------------------------------------------------------------
// Interpreta una riga completa ricevuta dal PC. Formato atteso: "U 0.250"
// -----------------------------------------------------------------------------
void handle_line() {
  buf[n_buf] = '\0';               // termino la stringa C (serve ad atof)
  if (buf[0] == 'U') {             // e' un comando per l'ESC?
    float u = atof(buf + 1);       // converto il testo dopo la 'U' in numero
    u_us = u_to_us(u);             // lo trasformo in microsecondi (con saturazione)
    esc.writeMicroseconds(u_us);   // aggiorno l'impulso: dal prossimo periodo vale il nuovo
    t_last_cmd = millis();         // segno che il PC e' vivo
  }
  n_buf = 0;                       // svuoto il buffer per la riga successiva
}

void setup() {
  Serial.begin(115200);            // seriale veloce: 80 righe/s da ~25 caratteri = ~20 kbit/s

  esc.attach(PIN_ESC, US_MIN, US_MAX);  // attivo il segnale sull'ESC
  esc.writeMicroseconds(US_MIN);        // impulso minimo: serve all'ESC per "armarsi"

  scale.begin(PIN_HX_DT, PIN_HX_SCK);   // inizializzo l'HX711 (canale A, guadagno 128)

  delay(3000);                     // 3 s a impulso minimo: l'ESC fa i suoi bip e si arma

  Serial.println(F("READY"));      // dico al PC che posso partire
  t_last_cmd = millis();           // faccio partire il watchdog da adesso
}

void loop() {
  // --- 1. Leggo i caratteri arrivati dal PC, senza mai bloccarmi ad aspettare ---
  while (Serial.available()) {               // finche' ci sono caratteri nel buffer seriale
    char c = Serial.read();                  // ne prendo uno
    if (c == '\n') {                         // fine riga -> comando completo
      handle_line();
    } else if (c != '\r' && n_buf < sizeof(buf) - 1) {
      buf[n_buf++] = c;                      // altrimenti lo accodo (ignoro '\r')
    }
  }

  // --- 2. Watchdog: se il PC non parla da troppo, spengo il motore ---
  if (millis() - t_last_cmd > WATCHDOG_MS && u_us != US_MIN) {
    u_us = US_MIN;
    esc.writeMicroseconds(US_MIN);
  }

  // --- 3. Se l'HX711 ha un campione nuovo, lo leggo e lo mando al PC ---
  if (scale.is_ready()) {          // DT basso = campione pronto (a 80 Hz: ogni 12.5 ms)
    unsigned long t = micros();    // timestamp in us: il PC ricostruira' il tempo da qui
    long raw = scale.read();       // valore grezzo a 24 bit (conteggi, non ancora Newton)
    // formato riga: tempo_us,raw,impulso_us
    Serial.print(t);   Serial.print(',');
    Serial.print(raw); Serial.print(',');
    Serial.println(u_us);
  }
}
