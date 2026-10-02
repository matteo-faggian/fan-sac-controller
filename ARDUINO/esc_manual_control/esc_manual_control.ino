#include <Servo.h>
Servo esc;  // Declare a Servo object named 'esc'

void setup() {
  // Start serial communication
  Serial.begin(9600);  
   esc.attach(6); 
  // Wait for serial monitor to open
  while (!Serial) {
    ;  // Do nothing until Serial is available
  }

  // Wait for initial input
  Serial.println("Premi 'C' per calibrare l'ESC o qualsiasi altra chiave per avviare il controllo.");

  // Wait for user input
  while (!Serial.available()) ;
  char scelta = Serial.read();

  if (scelta == 'C' || scelta == 'c') {
    calibraESC();  // Call the calibration function
  } else {
    Serial.println("Avvio modalità controllo...");
    esc.write(90);  // Set ESC to neutral (stop position)
    Serial.println("Inserisci un valore tra 0 e 180 (90 = stop)");
  }
}

void loop() {
  if (Serial.available() > 0) {
    int val = Serial.parseInt();  // Read the value sent from the serial monitor
    if (val >= 0 && val <= 180) {
      esc.write(val);  // Send the value to the ESC
      Serial.print("Valore inviato all'ESC: ");
      Serial.println(val);
    } else {
      Serial.println("⚠️ Inserisci un numero tra 0 e 180.");
    }
  }
}

void calibraESC() {
  Serial.println("=== CALIBRAZIONE ESC ===");
  Serial.println("1. Scollega la batteria dall'ESC.");
  Serial.println("2. Premi un tasto quando sei pronto.");

  while (!Serial.available())
    ;
  Serial.read();  // Read the incoming data

  Serial.println("3. Inviando segnale massimo (180)...");
  esc.write(180);  // Send the maximum signal to ESC (full throttle)
  delay(2000);  // Wait for ESC to respond

  Serial.println("4. Ora collega la batteria all'ESC.");
  Serial.println("   Attendi i beep, poi premi un tasto per continuare...");

  while (!Serial.available())
    ;
  Serial.read();  // Wait for user to press a key

  Serial.println("5. Inviando segnale minimo (0)...");
  esc.write(0);  // Send the minimum signal to ESC (full stop)
  delay(3000);  // Wait for ESC to respond

  Serial.println("✅ Calibrazione completata! Avvio modalità controllo...");
  Serial.println("Inserisci un valore tra 0 e 180 (90 = stop)");
}
