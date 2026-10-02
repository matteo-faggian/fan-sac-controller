// =============================================================================
// esc_calibrazione.ino — insegna all'ESC i fine corsa 1000 us (zero) e 2000 us (max)
// -----------------------------------------------------------------------------
// !!! TOGLI L'ELICA PRIMA DI INIZIARE !!!
// All'accensione l'ESC riceve il segnale di MASSIMO: se per qualche motivo non
// entrasse in modalita' calibrazione, il motore partirebbe a piena potenza.
//
// Come funziona la calibrazione (procedura standard della maggior parte degli ESC):
//   1. l'ESC si accende e vede il segnale MASSIMO  -> entra in modalita' calibrazione
//      e memorizza quell'impulso come "100%"
//   2. gli mandi il segnale MINIMO                 -> lo memorizza come "0%"
//   3. bip di conferma                             -> fine corsa salvati in memoria
//
// Uso: Monitor seriale a 115200 baud, terminatore "A capo (NL)".
// Dopo la calibrazione puoi provare la soglia di partenza con i tasti 0..9.
// =============================================================================

#include <Servo.h>   // genera l'impulso stile servo per l'ESC

const uint8_t PIN_ESC = 9;   // stesso pin dello sketch di acquisizione
const int US_MIN = 1000;     // impulso che diventera' "motore fermo"
const int US_MAX = 2000;     // impulso che diventera' "piena potenza"

Servo esc;                   // oggetto che genera l'impulso

void setup() {
  Serial.begin(115200);
  esc.attach(PIN_ESC, US_MIN, US_MAX);   // limiti dell'impulso: 1000..2000 us
  esc.writeMicroseconds(US_MAX);         // PARTO DAL MASSIMO: e' cosi' che l'ESC
                                         // capisce che vogliamo calibrare

  Serial.println(F("=== CALIBRAZIONE ESC ==="));
  Serial.println(F("1. Elica TOLTA? Batteria dell'ESC ancora SCOLLEGATA?"));
  Serial.println(F("2. Ora sto mandando 2000 us (massimo)."));
  Serial.println(F("3. Collega la batteria all'ESC: sentirai dei bip (massimo riconosciuto)."));
  Serial.println(F("4. SUBITO dopo quei bip (entro 2 s) scrivi 'm' e premi Invio."));
}

void loop() {
  if (!Serial.available()) return;   // nessun comando: non faccio niente
  char c = Serial.read();            // leggo un carattere

  if (c == 'm') {
    // --- Passo 2: segnale minimo -> l'ESC lo salva come "zero" ---
    esc.writeMicroseconds(US_MIN);
    Serial.println(F("Mandato 1000 us (minimo). Dovresti sentire bip di conferma."));
    Serial.println(F("Calibrazione fatta. Ora prova la soglia di partenza:"));
    Serial.println(F("  tasti 0..9 -> 1000, 1050, 1100, ... 1450 us   (0 = fermo)"));
  }
  else if (c >= '0' && c <= '9') {
    // --- Test della soglia: ogni cifra = +50 us, max 1450 us (45% della corsa) ---
    int us = US_MIN + (c - '0') * 50;   // '0' -> 1000, '1' -> 1050, ... '9' -> 1450
    esc.writeMicroseconds(us);
    Serial.print(F("Impulso: "));
    Serial.print(us);
    Serial.print(F(" us  (u = "));
    Serial.print((us - US_MIN) / 1000.0, 2);   // lo stesso u usato dallo script Python
    Serial.println(F(")"));
  }
}
