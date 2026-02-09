// ===================================================
// PROGETTO: Rilevamento Tosse
// MODALITÀ: DUAL RATE (Audio Calc 25ms / Send ~Fast)
// ===================================================

#include <Adafruit_TinyUSB.h> 
#include <Wire.h>              
#include "SparkFun_BMI270_Arduino_Library.h" 

// ===================================================
// 1. CONFIGURAZIONE
// ===================================================
const int micPin = A0;              
const int DC_OFFSET = 511;          // <-- TUO OFFSET
const int GAMMA_NOISE = 30;         
const float GAIN_FACTOR = 1.0;      

// ===================================================
// 2. VARIABILI SISTEMA
// ===================================================
const int fs = 2000;                
const unsigned long samplePeriod_us = 1000000 / fs; // 500us

// Audio Frame (Calcolo ogni 25ms)
const int frame_ms = 25;            
const int frame_len = fs * frame_ms / 1000; 

BMI270 imu;

// Audio Sampling Vars
unsigned long lastSampleTime = 0;
int16_t sigBuffer[frame_len];
int bufferIndex = 0;

// Variabili "Shared" (Ultimo valore calcolato)
// Queste vengono inviate ripetutamente finché non cambiano
float global_Energy = 0.0;
float global_ZCR = 0.0;

// Variabili IMU
float a_z = 0;      
float d_z = 0;      
long t_old = 0;
long t_last_send = 0;

// ===================================================
// SETUP
// ===================================================
void setup() {
  Serial.begin(500000); // 500k Baud per reggere l'alta frequenza
  delay(2000); 

  Wire.begin();
  Wire.setClock(400000); // I2C Fast Mode
  
  if (imu.beginI2C() != BMI2_OK) {
    while (1); 
  }
  t_old = millis();
}

// ===================================================
// LOOP
// ===================================================
void loop() {
  
  // -------------------------------------------------
  // TASK A: CAMPIONAMENTO AUDIO (Priorità Alta)
  // Deve girare ogni 500us.
  // -------------------------------------------------
  if (micros() - lastSampleTime >= samplePeriod_us) {
    lastSampleTime += samplePeriod_us;

    int rawVal = analogRead(micPin);
    int sig_centered = rawVal - DC_OFFSET;
    if (abs(sig_centered) < GAMMA_NOISE) sig_centered = 0;
    int16_t sig_gained = (int16_t)(sig_centered * GAIN_FACTOR);
    
    sigBuffer[bufferIndex++] = sig_gained;

    // Se il buffer è pieno (ogni 25ms), calcoliamo le nuove features
    if (bufferIndex >= frame_len) {
      // Aggiorniamo le variabili globali che verranno lette dal loop di invio
      global_ZCR = computeZCR(sigBuffer, frame_len);
      global_Energy = computeEnergy(sigBuffer, frame_len);
      bufferIndex = 0; // Reset buffer
    }
  }

  // -------------------------------------------------
  // TASK B: LETTURA IMU & INVIO DATI (As fast as possible)
  // Cerchiamo di non intasare troppo: inviamo ogni volta che leggiamo l'IMU
  // -------------------------------------------------
  
  // Leggiamo l'IMU. L'I2C impiega circa 1-2ms, quindi questo funge da limitatore naturale
  // Non usiamo delay() per non perdere campioni audio.
  
  // Eseguiamo questo blocco solo se NON stiamo per campionare l'audio (per sicurezza)
  if (micros() - lastSampleTime < (samplePeriod_us - 100)) {
     
     // 1. Leggi IMU e Calcola Jerk (Dato Fresco)
     imu.getSensorData();
     long currentMillis = millis();
     long delta = currentMillis - t_old;
     
     // Evitiamo divisioni per zero o delta troppo piccoli
     if (delta > 0) {
        float current_accel = imu.data.accelZ;
        d_z = abs((current_accel - a_z) / delta);
        a_z = current_accel;
        t_old = currentMillis;
     }

     // 2. Invia Dati (Mix di dati freschi e vecchi)
     // Limitiamo l'invio a circa 250Hz (ogni 4ms) per non saturare il buffer seriale
     // che potrebbe bloccare l'audio se si riempie.
     if (millis() - t_last_send >= 4) { 
         t_last_send = millis();
         
         Serial.print(global_Energy, 2); // Dato "vecchio" (aggiornato ogni 25ms)
         Serial.print(",");
         Serial.print(global_ZCR, 6);    // Dato "vecchio" (aggiornato ogni 25ms)
         Serial.print(",");
         Serial.print(d_z, 4);           // Dato "fresco" (aggiornato istantaneamente)
         Serial.print(",");
         Serial.println(0); 
     }
  }
}

// Funzioni Calcolo (Invariate)
float computeZCR(int16_t* frame, int len) {
  int signChanges = 0;
  for (int i = 1; i < len; i++) {
    if ((long)frame[i] * (long)frame[i - 1] < 0) signChanges++;
  }
  return (float)signChanges / (float)(len - 1);
}

float computeEnergy(int16_t* frame, int len) {
  unsigned long sum = 0;
  for (int i = 0; i < len; i++) { sum += abs(frame[i]); }
  return (float)sum / (float)len;
}