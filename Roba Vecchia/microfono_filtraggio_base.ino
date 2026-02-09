// =====================
// PARAMETERS
// =====================
const int micPin = A0;      // microfono
const int Gamma = 30;       // soglia segnale
const int sampleDelay = 1;  // ms tra campioni (~1 kHz)
const int frame_ms = 25;    // finestra ZCR in ms
const int fs = 1000;        // frequenza approssimativa di campionamento
#include <Adafruit_TinyUSB.h>
const int frame_len = fs * frame_ms / 1000;  // numero di campioni per frame

// =====================
// BUFFER
// =====================
int16_t sigBuffer[200]; // buffer per ZCR (dimensione >= frame_len)
int bufferIndex = 0;

// =====================
// UTILITY
// =====================

// calcola lo ZCR su un frame
float computeZCR(int16_t* frame, int len) {
  int signChanges = 0;
  int prevSign = (frame[0] >= 0) ? 1 : -1;
  for (int i = 1; i < len; i++) {
    int s = (frame[i] >= 0) ? 1 : -1;
    if (s != prevSign) signChanges++;
    prevSign = s;
  }
  return (float)signChanges / (float)len;
}

// =====================
// MAIN LOOP
// =====================
void setup() {
   // initialize serial communication at 9600 bits per second:
Serial.begin(115200);
// Give the host a chance to enumerate (up to 3 seconds) [ADDED]
unsigned long t0 = millis();
while (!Serial && millis() - t0 < 3000) {
// do nothing
}
}

void loop() {
  // leggi microfono centrato su 0
  int rawVal = analogRead(micPin);
  int offset = 119;
  int sig = rawVal -512 - offset;
  //int sig = rawVal- offset;

  // applica soglia
  if (abs(sig) < Gamma) sig = 0;

  // salva nel buffer
  sigBuffer[bufferIndex++] = sig;
  if (bufferIndex >= frame_len) bufferIndex = 0; // circolare

  // calcola ZCR solo se buffer pieno
  float zcr = 0.0;
  if (bufferIndex == 0) {
    zcr = computeZCR(sigBuffer, frame_len);
  }

  // stampa: segnale, soglia e ZCR
  Serial.print(sig);
  Serial.print("\t");
  Serial.print(Gamma);
  Serial.print("\t");
  Serial.println(zcr);

  delay(sampleDelay);
}