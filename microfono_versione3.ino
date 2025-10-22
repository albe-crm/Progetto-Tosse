#include <Adafruit_TinyUSB.h>
// =====================
// PARAMETRI AUDIO
// =====================
const int micPin = A0;      // Pin microfono
const int Gamma = 30;       // Soglia rumore (da calibrare)
const int DC_OFFSET = 631;  // Offset DC (da calibrare)

// =====================
// PARAMETRI DI CAMPIONAMENTO (fs = 2kHz)
// =====================
const int fs = 2000;
const unsigned long samplePeriod_us = 1000000 / fs;
unsigned long lastSampleTime = 0;

// =====================
// PARAMETRI FRAME (per ZCR & Energia)
// =====================
const int frame_ms = 20;
const int frame_len = fs * frame_ms / 1000;
int16_t sigBuffer[frame_len];
int bufferIndex = 0;

// ===================================================
// PARAMETRI: Filtro e Peak Detection
// ===================================================
const int HISTORY_SIZE = 10;          // Dimensione buffer storico
const int SMOOTH_WINDOW = 3;          // Finestra per media mobile

// --- Buffer per ZCR
float zcrHistory[HISTORY_SIZE];
int zcrHistoryIndex = 0;
float smoothedZCR = 0.0;

// --- NUOVO: Buffer per ENERGIA
float energyHistory[HISTORY_SIZE];
int energyHistoryIndex = 0;
float smoothedEnergy = 0.0;


// ===================================================
// *** SOGLIE CHIAVE DA CALIBRARE! ***
// ===================================================
// ZCR: deve essere sopra le vocali (es. > 0.2)
const float ZCR_COUGH_THRESHOLD = 0.1; 

// ENERGIA: deve essere sopra il parlato normale (es. > 50)
const float ENERGY_COUGH_THRESHOLD = 60.0; //DA VERIFICARE

bool isPeak = false;
// ===================================================

// =====================
// UTILITY - Calcolo ZCR (invariato)
// =====================
float computeZCR(int16_t* frame, int len) {
  int signChanges = 0;
  for (int i = 1; i < len; i++) {
    if ((long)frame[i] * (long)frame[i-1] < 0) {
      signChanges++;
    }
  }
  return (float)signChanges / (float)(len - 1);
}

// =====================
// NUOVA UTILITY - Calcolo Energia (Media Valori Assoluti)
// =====================
float computeEnergy(int16_t* frame, int len) {
  unsigned long sum = 0; // Usiamo unsigned long per evitare overflow
  for (int i = 0; i < len; i++) {
    sum += abs(frame[i]);
  }
  return (float)sum / (float)len;
}

// =====================
// SETUP
// =====================
void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
while (!Serial && millis() - t0 < 3000) {
// do nothing
}
  for(int i = 0; i < HISTORY_SIZE; i++) {
    zcrHistory[i] = 0.0;
    energyHistory[i] = 0.0; // Inizializza nuovo buffer
  }
}

// =====================
// MAIN LOOP
// =====================
void loop() {
  
  if (micros() - lastSampleTime >= samplePeriod_us) {
    lastSampleTime += samplePeriod_us;

    int rawVal = analogRead(micPin);
    int sig = rawVal - DC_OFFSET;
    if (abs(sig) < Gamma) sig = 0;

    sigBuffer[bufferIndex++] = sig;

    // ELABORAZIONE (solo se il buffer è pieno)
    if (bufferIndex >= frame_len) {
      
      // 4a. Calcola ZCR e ENERGIA
      float currentZCR = computeZCR(sigBuffer, frame_len);
      float currentEnergy = computeEnergy(sigBuffer, frame_len); // NUOVO
      bufferIndex = 0; 

      // 4b. Salva ZCR e ENERGIA nello storico
      zcrHistory[zcrHistoryIndex] = currentZCR;
      energyHistory[energyHistoryIndex] = currentEnergy; // NUOVO
      
      // 4c. FILTRO SMOOTHING (Media Mobile)
      smoothedZCR = 0.0;
      smoothedEnergy = 0.0; // NUOVO
      
      for (int i = 0; i < SMOOTH_WINDOW; i++) {
        int idx_zcr = (zcrHistoryIndex - i + HISTORY_SIZE) % HISTORY_SIZE;
        smoothedZCR += zcrHistory[idx_zcr];

        int idx_energy = (energyHistoryIndex - i + HISTORY_SIZE) % HISTORY_SIZE; // NUOVO
        smoothedEnergy += energyHistory[idx_energy]; // NUOVO
      }
      smoothedZCR /= (float)SMOOTH_WINDOW;
      smoothedEnergy /= (float)SMOOTH_WINDOW; // NUOVO

      // ===================================================
      // 4d. PEAK DETECTION (Logica modificata)
      // ===================================================
      // Un picco è valido SOLO SE ENTRAMBE le soglie sono superate
      if (smoothedZCR > ZCR_COUGH_THRESHOLD && smoothedEnergy > ENERGY_COUGH_THRESHOLD) {
        if (!isPeak) { 
          Serial.println(">>> PICCO ZCR+ENERGIA RILEVATO (TOSSE?) <<<");
          isPeak = true;
        }
      } else {
        if (isPeak) {
          Serial.println("--- Fine picco ---");
          isPeak = false;
        }
      }

      // 4e. Stampa per PLOTTER SERIALE (per calibrazione)
      float peakPlotValue = isPeak ? 1.0 : 0.0; 
      // Ora stampa ZCR filtrato E Energia filtrata
      //Serial.println(smoothedZCR);
      //Serial.print("\t");
      //Serial.println(smoothedEnergy); // Invia ZCR e Energia al plotter
      //Serial.print("isPeak");
      Serial.println(peakPlotValue);

      // Avanza indici
      zcrHistoryIndex = (zcrHistoryIndex + 1) % HISTORY_SIZE;
      energyHistoryIndex = (energyHistoryIndex + 1) % HISTORY_SIZE; // NUOVO
    }
  }
}