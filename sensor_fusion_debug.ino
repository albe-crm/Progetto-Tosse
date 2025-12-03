// ===================================================
// PROGETTO: Rilevamento Tosse (Sensor Fusion)
// VERSIONE: Debug con Avvisi Seriali
// ===================================================
#include <Adafruit_TinyUSB.h> 
#include <Wire.h>              
#include "SparkFun_BMI270_Arduino_Library.h" 


// PARAMETRI AUDIO Microfono
// ===================================================
const int micPin = A0;     
const int DC_OFFSET = 511; // Offset DC (DA RICALIBRARE se necessario)
const float GAIN_FACTOR = 3.0;
const int Gamma = 30;      

// Parametri di campionamento (fs = 2kHz)
const int fs = 2000;
const unsigned long samplePeriod_us = 1000000 / fs;
unsigned long lastSampleTime = 0;

// Parametri Frame (per ZCR & Energia)
const int frame_ms = 25;
const int frame_len = fs * frame_ms / 1000;
int16_t sigBuffer[frame_len];
int bufferIndex = 0;

// Filtro e Peak Detection
const int HISTORY_SIZE = 10;
const int SMOOTH_WINDOW = 3;

// Buffer per ZCR
float zcrHistory[HISTORY_SIZE];
int zcrHistoryIndex = 0;
float smoothedZCR = 0.0;

// Buffer per ENERGIA
float energyHistory[HISTORY_SIZE];
int energyHistoryIndex = 0;
float smoothedEnergy = 0.0;

// *** SOGLIE CHIAVE AUDIO ***
const float ZCR_COUGH_THRESHOLD = 0.01;
const float ENERGY_COUGH_THRESHOLD = 115.0; 

// ===================================================
// PARAMETRI IMU 
// ===================================================
#define TIME_INTGR 100
#define SOGLIA_TOSSE 0.15

// Oggetto sensore IMU
BMI270 imu;
uint8_t i2cAddress = BMI2_I2C_PRIM_ADDR; // 0x68

// Variabili per calcoli IMU
double a_x, a_y, a_z, d_x, d_y, d_z, delta, intgr_z = 0;
long t_old, timer = 0;

// VARIABILI PER IL LATCH TEMPORIZZATO IMU
const long IMU_PEAK_DURATION_MS = 500; 
unsigned long imuPeakStartTime = 0;    

// ===================================================
// STATI GLOBALI PER LA LOGICA COMBINATA
// ===================================================
bool mic_isPeak = false;  
bool imu_isTosse = false; 

// --- NUOVE VARIABILI PER IL CONTEGGIO ---
unsigned int coughCount = 0;  
bool isCountingCough = false; 

// ===================================================
// Funzioni Ausiliarie
// ===================================================

float computeZCR(int16_t* frame, int len) {
  int signChanges = 0;
  for (int i = 1; i < len; i++) {
    if ((long)frame[i] * (long)frame[i - 1] < 0) {
      signChanges++;
    }
  }
  return (float)signChanges / (float)(len - 1);
}

float computeEnergy(int16_t* frame, int len) {
  unsigned long sum = 0;
  for (int i = 0; i < len; i++) {
    sum += abs(frame[i]);
  }
  return (float)sum / (float)len;
}

// ===================================================
// SETUP 
// ===================================================
void setup() {
  Serial.begin(115200);
  delay(2000); // Piccola attesa per aprire il monitor
  Serial.println("--- AVVIO SISTEMA SENSOR FUSION ---");

  // Setup IMU
  Wire.begin();
  if (imu.beginI2C(i2cAddress) != BMI2_OK) {
    Serial.println("ERRORE CRITICO: BMI270 non trovato!");
    while(1);
  }
  Serial.println("BMI270 OK.");
  
  // Inizializza t_old per la logica IMU
  t_old = millis();
  timer = millis();

  // Reset Buffer
  for (int i = 0; i < HISTORY_SIZE; i++) {
    zcrHistory[i] = 0.0;
    energyHistory[i] = 0.0;
  }
  
  Serial.println("Sistema Pronto. In attesa di tosse...");
}

// ===================================================
// MAIN LOOP 
// ===================================================
void loop() {
  // ===================================================
  // 1. LOGICA IMU (con Latch Temporizzato)
  // ===================================================

  imu.getSensorData(); 

  delta = millis() - t_old;
  // Protezione divisione per zero
  if (delta == 0) return; 
  
  t_old = millis();

  d_z = (imu.data.accelZ - a_z) / delta;
  a_z = imu.data.accelZ;

  intgr_z += abs(d_z);

  // --- A. Logica di START Picco (ogni 100ms) ---
  if (millis() - timer >= TIME_INTGR) {

    // Se l'integrale supera la soglia E non c'è già un picco attivo...
    if ((intgr_z > SOGLIA_TOSSE) && (!imu_isTosse)) {
      // ... questo è un NUOVO picco IMU.
      imu_isTosse = true;
      imuPeakStartTime = millis(); 
      
      // >>> AVVISO IMU <<<
      Serial.println(">> [IMU] Movimento Rilevato (Latch Attivo)"); 
    }
    // Azzera integrale e ri-inizializza il timer
    timer = millis();
    intgr_z = 0;
  }

  // --- B. Logica di STOP Picco (continua) ---
  if (imu_isTosse) {
    if (millis() - imuPeakStartTime >= IMU_PEAK_DURATION_MS) {
      imu_isTosse = false;
      // Serial.println(">> [IMU] Latch Scaduto"); // Decommenta se vuoi vedere quando finisce l'attesa
    }
  }
  
  // ===================================================
  // 2. ESEGUI LOGICA AUDIO 
  // ===================================================
  
  if (micros() - lastSampleTime >= samplePeriod_us) {
    lastSampleTime += samplePeriod_us;

    int rawVal = analogRead(micPin);
    int sig_centered = rawVal - DC_OFFSET;
    if (abs(sig_centered) < Gamma) {
      sig_centered = 0;
    }
    int16_t sig_gained = (int16_t)(sig_centered * GAIN_FACTOR);
    sigBuffer[bufferIndex++] = sig_gained;

    // 3. ELABORAZIONE FRAME (Ogni 25ms)
    if (bufferIndex >= frame_len) {

      float currentZCR = computeZCR(sigBuffer, frame_len);
      float currentEnergy = computeEnergy(sigBuffer, frame_len);
      bufferIndex = 0;

      // Aggiorna buffer di cronologia
      zcrHistory[zcrHistoryIndex] = currentZCR;
      energyHistory[energyHistoryIndex] = currentEnergy;

      // Smoothing
      smoothedZCR = 0.0;
      smoothedEnergy = 0.0;
      for (int i = 0; i < SMOOTH_WINDOW; i++) {
        int idx_zcr = (zcrHistoryIndex - i + HISTORY_SIZE) % HISTORY_SIZE;
        int idx_energy = (energyHistoryIndex - i + HISTORY_SIZE) % HISTORY_SIZE;
        smoothedZCR += zcrHistory[idx_zcr];
        smoothedEnergy += energyHistory[idx_energy];
      }
      smoothedZCR /= (float)SMOOTH_WINDOW;
      smoothedEnergy /= (float)SMOOTH_WINDOW;

      // Logica di Peak Detection Audio
      if (smoothedZCR > ZCR_COUGH_THRESHOLD && smoothedEnergy > ENERGY_COUGH_THRESHOLD) {
        if (!mic_isPeak) {
          mic_isPeak = true;
          // >>> AVVISO MICROFONO <<<
          Serial.println(">> [MIC] Audio Sospetto Rilevato");
        }
      } else {
        if (mic_isPeak) {
          mic_isPeak = false;
        }
      }

      // --- 4. LOGICA COMBINATA ---
      
      bool combined_cough = mic_isPeak && imu_isTosse;

      if (combined_cough) {
        if (!isCountingCough) {
          // >>> AVVISO FUSIONE AVVENUTA <<<
          Serial.println(">>> [ALLARME] TOSSE CONFERMATA! (Audio + IMU) <<<");
          
          coughCount++; 
          isCountingCough = true; 
          Serial.print("Conteggio Totale: ");
          Serial.println(coughCount);
        }
      } else {
        isCountingCough = false;
      }

      // Avanza gli indici della cronologia
      zcrHistoryIndex = (zcrHistoryIndex + 1) % HISTORY_SIZE;
      energyHistoryIndex = (energyHistoryIndex + 1) % HISTORY_SIZE;
    }
  }
}