#include <Adafruit_TinyUSB.h>
// =====================
// PARAMETRI AUDIO
// =====================
const int micPin = A0;      // Pin microfono
const int DC_OFFSET = 631;  // Offset DC (DA RICALIBRARE)

// -----------------------------------------------------------------
// GUADAGNO SOFTWARE
// -----------------------------------------------------------------
// Aumenta questo valore per amplificare il segnale digitalmente
// (es. 1.0 = nessun guadagno, 2.0 = doppio, 3.0 = triplo)
const float GAIN_FACTOR = 2.0; 
// -----------------------------------------------------------------

// Gamma lavora sul segnale prima del guadagno
const int Gamma = 30;       // Soglia rumore (DA RICALIBRARE)

// =====================
// PARAMETRI DI CAMPIONAMENTO (fs = 2kHz)
// =====================
const int fs = 2000;
const unsigned long samplePeriod_us = 1000000 / fs;
unsigned long lastSampleTime = 0;

// =====================
// PARAMETRI FRAME (per ZCR & Energia)
// =====================
const int frame_ms = 25;
const int frame_len = fs * frame_ms / 1000;
int16_t sigBuffer[frame_len];
int bufferIndex = 0;

// ===================================================
// PARAMETRI: Filtro e Peak Detection
// ===================================================
const int HISTORY_SIZE = 10;
const int SMOOTH_WINDOW = 3;

// --- Buffer per ZCR
float zcrHistory[HISTORY_SIZE];
int zcrHistoryIndex = 0;
float smoothedZCR = 0.0;

// --- Buffer per ENERGIA
float energyHistory[HISTORY_SIZE];
int energyHistoryIndex = 0;
float smoothedEnergy = 0.0;

// ===================================================
// *** SOGLIE CHIAVE DA RICALIBRARE! ***
// ===================================================
// ZCR: deve essere sopra le vocali (es. > 0.2)
const float ZCR_COUGH_THRESHOLD = 0.1; 

// ENERGIA: Questo valore è quasi sicuramente SBAGLIATO ora.
const float ENERGY_COUGH_THRESHOLD = 100.0; // DA RICALIBRARE!

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
// UTILITY - Calcolo Energia (invariato)
// =====================
float computeEnergy(int16_t* frame, int len) {
  unsigned long sum = 0;
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
    energyHistory[i] = 0.0;
  }
}

// =====================
// MAIN LOOP (Con Guadagno Software)
// =====================
void loop() {
  
  if (micros() - lastSampleTime >= samplePeriod_us) {
    lastSampleTime += samplePeriod_us;

    // 1. Leggi e centra
    int rawVal = analogRead(micPin);
    int sig_centered = rawVal - DC_OFFSET;

    // 2. Applica soglia Gamma (sul segnale originale)
    if (abs(sig_centered) < Gamma) {
      sig_centered = 0;
    }

    // 3. Applica GUADAGNO SOFTWARE
    int16_t sig_gained = (int16_t)(sig_centered * GAIN_FACTOR); 
    
    // 4. Salva nel buffer il segnale amplificato
    sigBuffer[bufferIndex++] = sig_gained;


    // --- ELABORAZIONE FRAME (invariata) ---
    if (bufferIndex >= frame_len) {
      
      float currentZCR = computeZCR(sigBuffer, frame_len);
      float currentEnergy = computeEnergy(sigBuffer, frame_len);
      bufferIndex = 0; 

      // ... (il resto della logica di smoothing e peak detection è identico) ...
      zcrHistory[zcrHistoryIndex] = currentZCR;
      energyHistory[energyHistoryIndex] = currentEnergy;
      
      smoothedZCR = 0.0;
      smoothedEnergy = 0.0;
      
      for (int i = 0; i < SMOOTH_WINDOW; i++) {
        int idx_zcr = (zcrHistoryIndex - i + HISTORY_SIZE) % HISTORY_SIZE;
        smoothedZCR += zcrHistory[idx_zcr];
        int idx_energy = (energyHistoryIndex - i + HISTORY_SIZE) % HISTORY_SIZE;
        smoothedEnergy += energyHistory[idx_energy];
      }
      smoothedZCR /= (float)SMOOTH_WINDOW;
      smoothedEnergy /= (float)SMOOTH_WINDOW;

      // Logica di Peak Detection (invariata)
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

      zcrHistoryIndex = (zcrHistoryIndex + 1) % HISTORY_SIZE;
      energyHistoryIndex = (energyHistoryIndex + 1) % HISTORY_SIZE;
    }
  }
}